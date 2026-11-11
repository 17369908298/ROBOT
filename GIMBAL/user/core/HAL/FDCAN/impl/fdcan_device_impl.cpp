#include "fdcan_device_impl.hpp"

namespace HAL::FDCAN
{

// 根据接收FIFO选择过滤器的分配目标
static uint32_t fifo_to_filter_config(uint32_t fifo)
{
    return (fifo == FDCAN_RX_FIFO1) ? FDCAN_FILTER_TO_RXFIFO1 : FDCAN_FILTER_TO_RXFIFO0;
}

// 根据接收FIFO选择非匹配帧的接收FIFO
static uint32_t fifo_to_accept_config(uint32_t fifo)
{
    return (fifo == FDCAN_RX_FIFO1) ? FDCAN_ACCEPT_IN_RX_FIFO1 : FDCAN_ACCEPT_IN_RX_FIFO0;
}

// 根据接收FIFO选择新消息中断
static uint32_t fifo_to_notification(uint32_t fifo)
{
    return (fifo == FDCAN_RX_FIFO1) ? FDCAN_IT_RX_FIFO1_NEW_MESSAGE : FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
}

// FdcanDevice实现
FdcanDevice::FdcanDevice(FDCAN_HandleTypeDef *handle, uint32_t filter_index, uint32_t fifo)
    : handle_(handle), filter_index_(filter_index), fifo_(fifo)
{
}

void FdcanDevice::init()
{
    configure_filter();
}

void FdcanDevice::start()
{
    if (handle_ == nullptr)
    {
        return;
    }

    HAL_FDCAN_Start(handle_);

    // 设置中断
    HAL_FDCAN_ActivateNotification(handle_, fifo_to_notification(fifo_), 0);
}

bool FdcanDevice::send(const Frame &frame)
{
    if (handle_ == nullptr)
    {
        return false;
    }

    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.Identifier = frame.id;
    tx_header.IdType = frame.is_extended_id ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx_header.TxFrameType = frame.is_remote_frame ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    tx_header.DataLength = IFdcanDevice::to_dlc_code(frame.dlc);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = static_cast<uint8_t>(frame.mailbox);

    return HAL_FDCAN_AddMessageToTxFifoQ(handle_, &tx_header, frame.data) == HAL_OK;
}

bool FdcanDevice::receive(Frame &frame)
{
    if (handle_ == nullptr || HAL_FDCAN_GetRxFifoFillLevel(handle_, fifo_) == 0)
    {
        return false;
    }

    FDCAN_RxHeaderTypeDef rx_header;
    if (HAL_FDCAN_GetRxMessage(handle_, fifo_, &rx_header, frame.data) != HAL_OK)
    {
        return false;
    }

    // 填充Frame结构体
    frame.id = IFdcanDevice::extract_id(rx_header);
    frame.dlc = IFdcanDevice::to_data_length(rx_header.DataLength);
    frame.is_extended_id = (rx_header.IdType == FDCAN_EXTENDED_ID);
    frame.is_remote_frame = (rx_header.RxFrameType == FDCAN_REMOTE_FRAME);

    // 自动触发所有注册的回调函数
    trigger_rx_callbacks(frame);

    return true;
}

FDCAN_HandleTypeDef *FdcanDevice::get_handle() const
{
    return handle_;
}

void FdcanDevice::configure_filter()
{
    if (handle_ == nullptr)
    {
        return;
    }

    FDCAN_FilterTypeDef filter;
    filter.IdType = FDCAN_STANDARD_ID;                  // 标准帧
    filter.FilterIndex = filter_index_;                 // 过滤器序号
    filter.FilterType = FDCAN_FILTER_MASK;              // 掩码模式
    filter.FilterConfig = fifo_to_filter_config(fifo_); // 匹配后存入的FIFO
    filter.FilterID1 = 0x0;                             // 掩码为0，接收所有ID
    filter.FilterID2 = 0x0;

    HAL_FDCAN_ConfigFilter(handle_, &filter);

    // 非匹配帧同样存入接收FIFO，远程帧直接拒收
    HAL_FDCAN_ConfigGlobalFilter(handle_,
                                 fifo_to_accept_config(fifo_),
                                 fifo_to_accept_config(fifo_),
                                 FDCAN_REJECT_REMOTE,
                                 FDCAN_REJECT_REMOTE);
}

void FdcanDevice::register_rx_callback(RxCallback callback)
{
    if (callback)
    {
        rx_callbacks_.push_back(callback);
    }
}

void FdcanDevice::trigger_rx_callbacks(const Frame &frame)
{
    for (auto &callback : rx_callbacks_)
    {
        if (callback)
        {
            callback(frame);
        }
    }
}

} // namespace HAL::FDCAN
