/**
 ******************************************************************************
 * @file    uart.h
 * @brief   Header file for UART (Universal Asynchronous Receiver/Transmitter) Class
 * @author  Robotics Team 2023, IOE Pulchowk Campus
 ******************************************************************************
 */

#ifndef UART_H__
#define UART_H__

#include "Robotlib/crypto/crc.hpp"
#include "usart.h"
#include "definition.h"

#define UART_START_BYTE     0xA5 
#define MAX_RECEIVING_DATA_SIZE   50  // Actual data size removed start byte and hash from packet.
#define MAX_TRANSMIT_DATA_SIZE    1   // Actual data size removed start byte and hash from packet.

/**
 * @brief Enumeration defining different states of receiving a byte over UART
 */
enum UARTReceiveByteStatus
{
  WAITING_FOR_START_BYTE, /**< Waiting for the start byte */
  WAITING_FOR_LEN,
  RECEIVING_DATA,         /**< Receiving data */
  WAITING_FOR_REM         /**< Waiting for remaining data */
};

/**
 * @brief 
 * 
 */
enum UARTStatus
{
  DISCONNECTED,         /**< UART disconnected status */
  HASH_DIDNT_MATCH,     /**< Hash mismatch status */
  ID_DIDNT_MATCH,
  OK                    /**< Successful UART communication status */
};

/**
 * @brief Enumeration defining different modes of UART operation
 */
enum UARTMode
{
  RECEIVING,    /**< UART in receiving mode */
  TRANSMITTING, /**< UART in transmitting mode */
  BOTH          /**< UART in both receiving and transmitting mode */
};

/**
 * @brief Class representing the UART communication functionality
 */
class UART
{
public:
  UART() = delete; /**< Delete default constructor to prevent _huart=null. */ 
  UART(UART_HandleTypeDef *, const uint8_t, const UARTMode);
  UART(UART_HandleTypeDef *, const uint8_t, const uint8_t, const UARTMode);

  UART(UART &&) = default;
  UART(const UART &) = default;
  UART &operator=(UART &&) = default;
  UART &operator=(const UART &) = default;
  ~UART() {}

  UART_HandleTypeDef *huart; /**< Pointer to the UART handle */

#ifdef __IMPLEMENT_CRC__
  CRC_Hash crc{7}; /**< CRC hash object */
#endif

  uint32_t last_updated_tick = 0; /**< Milliseconds since last update */

  uint8_t first_byte = 0, rem_byte = 0; /**< Stored first and last bytes from UART */

  uint8_t receiving_data_dma[MAX_RECEIVING_DATA_SIZE]; /**< Array to receive data*/

  uint8_t transmission_data[MAX_TRANSMIT_DATA_SIZE + 3]; /** Array to transmit start byte, data, hasg and line-feed (\n). */
  uint8_t r_size, t_size; /**< Sizes for received and transmitted data */

  uint8_t len;
  uint8_t id;

  UARTReceiveByteStatus receive_state = WAITING_FOR_START_BYTE; /**< Current state of receiving */

  UARTStatus status = DISCONNECTED; /**< Current UART status */

  UARTMode mode = RECEIVING; /**< Current UART mode */

  /**
   * @brief Get received data from UART
   * @param[out] Pointer to the received data
   * @return Status of the received data
   */
  UARTStatus get_received_data(uint8_t *);

  /**
   * @brief Receive data over UART
   * @return Status of the UART reception
   */
  UARTStatus receive();

  /**
   * @brief Transmit data over UART
   * @param[in] Pointer to the data to be transmitted
   */
  void transmit(uint8_t *);

  /**
   * @brief Initialize UART communication
   */
  void init();

  void display();

#ifndef __IMPLEMENT_CRC__
  uint8_t get_checksum(uint8_t* data, uint8_t size);
#endif
};

#endif // !_UART_H_
