#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "io_macro.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_can.h"
#include "pcanpro_variant.h"

#include "can.h"
#include "utils.h"
#include "debug.h"

#define CAN_WITHOUT_ISR 1

#define CAN_TX_FIFO_SIZE (272)
static struct t_can_dev
{
  void *dev;
  uint32_t tx_msgs;
  uint32_t tx_errs;
  uint32_t tx_ovfs;

  uint32_t rx_msgs;
  uint32_t rx_errs;
  uint32_t rx_ovfs;

  struct t_can_msg tx_fifo[CAN_TX_FIFO_SIZE];
  uint32_t tx_head;
  uint32_t tx_tail;
  uint32_t esr_reg;
  int (*rx_isr)( uint8_t, struct  t_can_msg* );
  int (*tx_isr)( uint8_t, struct  t_can_msg* );
  void (*err_handler)( int bus, uint32_t esr );
}
can_dev_array[CAN_BUS_TOTAL] = 
{
  [CAN_BUS_1] = { .dev = NULL },
};

static uint32_t g_fdcan_mode = FDCAN_MODE_NORMAL;
static bool g_iso_enable = true;

static int pcan_can_isr_frame(FDCAN_RxHeaderTypeDef *phdr, uint8_t* pData);


uint32_t pcan_can_msg_time( const struct t_can_msg *pmsg, uint32_t nt, uint32_t dt )
{
  const uint32_t data_bits = pmsg->size<<3;
  const uint32_t control_bits = ( pmsg->flags & MSG_FLAG_EXT ) ? 67:47;
 
  if( pmsg->flags & MSG_FLAG_BRS )
    return (control_bits*nt) + (data_bits*dt);
  else
    return (control_bits+data_bits)*nt;
}

int pcan_can_set_filter_mask( int bus, int num, int format, uint32_t id, uint32_t mask )
{
	eFeedback eRet = FBK_Success;
	bool extended = false;
	uint32_t filter = id;
	
	if(format == MSG_FLAG_EXT)
	{
		extended = true;
	}
	
	eRet = can_set_mask_filter(extended, filter, mask);
	if(eRet != FBK_Success)
	{
		PRINT_FAULT("can_set_mask_filter Error");
		return -1;
	}
	
	return 0;
}

int pcan_can_filter_init_stdid_list( int bus, const uint16_t *id_list, int id_len  )
{
  FDCAN_HandleTypeDef *p_can = can_dev_array[bus].dev;

  if( !p_can )
    return 0;
 
  return id_len;
}

//FCAN底层发送函数
static int _can_send( FDCAN_HandleTypeDef *p_can, struct t_can_msg *p_msg )
{
	FDCAN_TxHeaderTypeDef tx_header;
	uint32_t byte_count = 0;
	
	bool tx_fifo_full = can_is_tx_fifo_full();
	bool tx_allowed = can_is_tx_allowed();
	
	//CANFD控制器内部发送队列满或者当前不允许发送就直接退出
	if( tx_fifo_full || (tx_allowed != FBK_Success) )
	{
		PRINT_FAULT("Tx is not allowed! %d %d", tx_fifo_full, tx_allowed);
		return -1;
	}
		
	tx_header.TxFrameType         = FDCAN_DATA_FRAME;
	tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
	tx_header.IdType              = FDCAN_STANDARD_ID;
	tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
	tx_header.TxEventFifoControl  = FDCAN_STORE_TX_EVENTS;
	tx_header.ErrorStateIndicator = can_is_passive() ? FDCAN_ESI_PASSIVE : FDCAN_ESI_ACTIVE;

	if ( p_msg->flags & MSG_FLAG_EXT)
	{
	     tx_header.IdType     = FDCAN_EXTENDED_ID;
	     tx_header.Identifier = p_msg->id & 0x1FFFFFFF;
	}
	else
	{
		tx_header.Identifier = p_msg->id & 0x7FF;
	}

	if (p_msg->flags & MSG_FLAG_RTR)
	{
		tx_header.TxFrameType = FDCAN_REMOTE_FRAME;
	}

	byte_count = p_msg->size;
	
	if (p_msg->flags & MSG_FLAG_FD) // FDF bit is set if recessive
	{
	    tx_header.FDFormat = FDCAN_FD_CAN;
		if (p_msg->flags & MSG_FLAG_BRS)
		{
			tx_header.BitRateSwitch = FDCAN_BRS_ON;
		}

		/*if ( p_msg->flags & MSG_FLAG_ESI)
		{
			tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		}*/

		byte_count = utils_byte_count_to_dlc(byte_count);
	}

	// Shift bits up for direct storage in FIFO register
	// It is stupid that ST Microelectronics did not define a processor independent macro for this shift operation.
	// Will other processors also need this to be shifted 16 bits up ??
	tx_header.DataLength = (byte_count & 0xF) << 16;

	// Transmit CAN packet
	if (!can_send_packet(&tx_header, (void*)p_msg->data))
	{
	    return -1;
	}

	return 0;
}

static void pcan_can_flush_tx( int bus )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  struct t_can_msg *p_msg;

  /* empty fifo */
  if( p_dev->tx_head == p_dev->tx_tail )
    return;

  if( !p_dev->dev )
    return;
  
  p_msg = &p_dev->tx_fifo[p_dev->tx_tail];
  if( _can_send( p_dev->dev, p_msg ) < 0 )
    return;

  if( p_dev->tx_isr )
  {
    (void)p_dev->tx_isr( bus, p_msg );
  }

  /* update fifo index */
  p_dev->tx_tail = (p_dev->tx_tail+1)&(CAN_TX_FIFO_SIZE-1);
}

int pcan_can_write( int bus, struct t_can_msg *p_msg )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];

  if( !p_dev )
    return 0;

  if( !p_msg )
    return 0;

  uint32_t  tx_head_next = (p_dev->tx_head+1)&(CAN_TX_FIFO_SIZE-1);
  /* overflow ? just skip it */
  if( tx_head_next == p_dev->tx_tail )
  {
  	PRINT_FAULT("Tx queue overflow");
    ++p_dev->tx_ovfs;
    return -1;
  }

  p_dev->tx_fifo[p_dev->tx_head] = *p_msg;
  p_dev->tx_head = tx_head_next;

  return 0;
}

void pcan_can_install_rx_callback( int bus, int (*cb)( uint8_t, struct  t_can_msg* ) )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_dev->rx_isr = cb;
}

void pcan_can_install_tx_callback( int bus, int (*cb)( uint8_t, struct  t_can_msg* ) )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_dev->tx_isr = cb;
}

void pcan_can_install_err_callback( int bus, void (*cb)( int , uint32_t ) )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_dev->err_handler = cb;
}

int pcan_can_init_ex( int bus, uint32_t bitrate )
{
	struct t_can_dev *p_dev = &can_dev_array[bus];
	(void)bitrate;

	//初始化CANFD驱动
	can_init();

	//注册CANFD接收回调函数
	can_install_rx_callback(pcan_can_isr_frame);

	//初始化FDCAN句柄
	p_dev->dev = can_get_handle();
	
	return 0;
}

void pcan_can_set_silent( int bus, uint8_t silent_mode )
{
	FDCAN_HandleTypeDef *p_can = can_dev_array[bus].dev;

	if( !p_can )
		return;

	if(silent_mode)
	{
		g_fdcan_mode = FDCAN_MODE_BUS_MONITORING;
	}
	else
	{
		g_fdcan_mode = FDCAN_MODE_NORMAL;
	}
}

void pcan_can_set_iso_mode( int bus, uint8_t iso_mode )
{
	g_iso_enable = !iso_mode;
}

void pcan_can_set_loopback( int bus, uint8_t loopback )
{
}

void pcan_can_set_bus_active( int bus, uint16_t mode )
{
	eFeedback eRet = FBK_Success;
	FDCAN_HandleTypeDef *p_can = can_dev_array[bus].dev;
	
	if( !p_can )
		return;

	if( mode )
	{
		bool is_canfd = can_using_FD();
				
		eRet = can_open(g_fdcan_mode);
		if(eRet != FBK_Success)
		{
			PRINT_FAULT("can_open Error");
			return;
		}

		if(is_canfd)
		{
			PRINT_DEBUG("can_enable_ISO_mode(%d)", g_iso_enable);
			eRet = can_enable_ISO_mode(g_iso_enable);
			if(eRet != FBK_Success)
			{
				PRINT_FAULT("can_enable_ISO_mode Error");
				return;
			}
		}
		
	}
	else
	{
		can_close();
	}
}

/* set predefined best values */
void pcan_can_set_bitrate( int bus, uint32_t bitrate, int is_data_bitrate )
{
	eFeedback eRet = FBK_Success;
	FDCAN_HandleTypeDef *p_can = can_dev_array[bus].dev;
	can_nom_bitrate nom_bitrate = CAN_BITRATE_500K;
	can_data_bitrate data_bitrate = CAN_DATA_BITRATE_2M;
	
	if( !p_can )
		return;

	//设置仲裁波特率
	if(!is_data_bitrate)
	{
		switch(bitrate)
		{
			case 10000:
				nom_bitrate = CAN_BITRATE_10K;
				break;
			case 20000:
				nom_bitrate = CAN_BITRATE_20K;
				break;
			case 50000:
				nom_bitrate = CAN_BITRATE_50K;
				break;
			case 100000:
				nom_bitrate = CAN_BITRATE_100K;
				break;
			case 125000:
				nom_bitrate = CAN_BITRATE_125K;
				break;
			case 250000:
				nom_bitrate = CAN_BITRATE_250K;
				break;
			case 500000:
				nom_bitrate = CAN_BITRATE_500K;
				break;
			case 1000000:
				nom_bitrate = CAN_BITRATE_1000K;
				break;
			default:
				nom_bitrate = CAN_BITRATE_500K;
				break;
		}

		//仅设置到内部全局变量中,在can_open()生效
		eRet = can_set_baudrate(nom_bitrate);
		if(eRet != FBK_Success)
		{
			PRINT_FAULT("can_set_baudrate Error");
			return;
		}
	}
	else
	{		
		//设置CANFD的数据波特率
		switch(bitrate)
		{
			case 500000:
				data_bitrate = CAN_DATA_BITRATE_500K;
				break;
			case 1000000:
				data_bitrate = CAN_DATA_BITRATE_1M;
				break;
			case 2000000:
				data_bitrate = CAN_DATA_BITRATE_2M;
				break;
			case 4000000:
				data_bitrate = CAN_DATA_BITRATE_4M;
				break;
			case 5000000:
				data_bitrate = CAN_DATA_BITRATE_5M;
				break;
			case 8000000:
				data_bitrate = CAN_DATA_BITRATE_8M;
				break;
			default:
				data_bitrate = CAN_DATA_BITRATE_2M;
				break;
		}

		//仅设置到内部全局变量中,在can_open()生效
		eRet = can_set_data_baudrate(data_bitrate);
		if(eRet != FBK_Success)
		{
			PRINT_FAULT("can_set_data_baudrate Error");
			return;
		}
	}

}

//该接口需同时改变CANFD时钟频率pcan_can_set_canfdclock(pcan_device.can[channel].can_clock)
void pcan_can_set_bitrate_ex( int bus, uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw, int is_data_bitrate )
{
	eFeedback eRet = FBK_Success;
	
	if(is_data_bitrate)
		eRet = can_set_data_bit_timing(brp, tseg1, tseg2, sjw);
	else
		eRet = can_set_nom_bit_timing (brp, tseg1, tseg2, sjw);

	if(eRet != FBK_Success)
	{
		PRINT_FAULT("pcan_can_set_bitrate_ex error");
		return;
	}
}

static void pcan_can_tx_complete( int bus, int mail_box )
{
  ++can_dev_array[bus].tx_msgs;
}

static void pcan_can_tx_err( int bus, int mail_box )
{
  ++can_dev_array[bus].tx_errs;
}

int pcan_can_stats( int bus, struct t_can_stats *p_stats )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  
  p_stats->tx_msgs = p_dev->tx_msgs;
  p_stats->tx_errs = p_dev->tx_errs;
  p_stats->rx_msgs = p_dev->rx_msgs;
  p_stats->rx_errs = p_dev->rx_errs;
  p_stats->rx_ovfs = p_dev->rx_ovfs;

  return sizeof( struct t_can_stats );
}

void pcan_can_poll( void )
{
  static uint32_t err_last_check = 0;
  uint32_t ts_ms;
  
  ts_ms = pcan_timestamp_millis();
  
  //CANFD消息处理循环
  can_process( ts_ms );
  
  pcan_can_flush_tx( CAN_BUS_1 );

  if( (uint32_t)( err_last_check - ts_ms ) > 250 )
  {
    err_last_check = ts_ms;
    for( int i = 0; i < CAN_BUS_TOTAL; i++ )
    {
      if( !can_dev_array[i].err_handler )
        continue;
      FDCAN_HandleTypeDef *pcan = can_dev_array[i].dev;
      if( !pcan )
        continue;
      /*if( can_dev_array[i].esr_reg != pcan->Instance->ESR )
      {
        can_dev_array[i].esr_reg = pcan->Instance->ESR;
        can_dev_array[i].err_handler( i, can_dev_array[i].esr_reg );
      }*/
    }
  }
}

/* --------------- HAL PART ------------- */
static int _bus_from_int_dev( FDCAN_GlobalTypeDef *can )
{
  if( can == FDCAN1 )
    return CAN_BUS_1;
	
  /* abnormal! */
  return CAN_BUS_1;
}

int pcan_can_isr_frame(FDCAN_RxHeaderTypeDef *phdr, uint8_t* pData)
{
	uint8_t bus = 0;
	struct t_can_dev * const p_dev = &can_dev_array[bus];
	struct t_can_msg  msg = { 0 };
	
	if(phdr->IdType == FDCAN_EXTENDED_ID)
	{
		msg.flags |= MSG_FLAG_EXT;
		msg.id = phdr->Identifier & 0x1FFFFFFF;
	}
	else if(phdr->IdType == FDCAN_STANDARD_ID)
	{
		msg.flags |= MSG_FLAG_STD;
		msg.id = phdr->Identifier & 0x000007FF;
	}
	
	if(phdr->RxFrameType == FDCAN_REMOTE_FRAME)
	{
		msg.flags |= MSG_FLAG_RTR;
	}
	
	//经典CAN的数据长度
	msg.size = (phdr->DataLength >> 16) & 0xF;
	
	if(phdr->FDFormat == FDCAN_FD_CAN)
	{
		msg.flags |= MSG_FLAG_FD;
		if(phdr->BitRateSwitch == FDCAN_BRS_ON)
		{
			msg.flags |= MSG_FLAG_BRS;
		}

		if(phdr->ErrorStateIndicator == FDCAN_ESI_PASSIVE)
		{
			msg.flags |= MSG_FLAG_ESI;
		}

		//需要转换CANFD的数据长度
		msg.size = utils_dlc_to_byte_count(msg.size);
	}

	if(msg.size > CAN_PAYLOAD_MAX_SIZE)
		return -1;
			
	msg.timestamp = pcan_timestamp_us();
	memcpy(msg.data, pData, msg.size);
	
	if( p_dev->rx_isr )
	{
		if( p_dev->rx_isr( bus, &msg ) < 0 )
		{
			++p_dev->rx_ovfs;
			return -1;
		}
	}
	++p_dev->rx_msgs;

	return 0;
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
	pcan_can_tx_complete( _bus_from_int_dev( hfdcan->Instance ), BufferIndexes );
}



