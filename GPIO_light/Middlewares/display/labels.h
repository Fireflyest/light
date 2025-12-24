#ifndef __LABELS_H
#define __LABELS_H

#include "stm32f4xx.h"


/**
 * @brief 跳零位存储结构，最多存储32位
 */
typedef struct {
    uint32_t buffer;             // 使用1个32位整数存储32位
    uint8_t currentIndex;        // 当前操作位置 (0-31)
    uint8_t readIndex;           // 读取位置 (0-31)
    uint8_t effectiveLength;     // 有效长度 (最后一个非零位的索引+1)
    uint8_t firstNonZeroIndex;   // 第一个非零值的索引 (0-31)，如果全为零则为0xFF
} SkipZeroBitStore;

/**
 * @brief 初始化跳零位存储结构
 * @param store 存储结构指针
 */
void Labels_Init(SkipZeroBitStore* store);

/**
 * @brief 获取第一个非零值之前的零值数量
 * @param store 存储结构指针
 * @return 第一个非零值之前的零值数量
 */
uint8_t Labels_GetLeadingZeros(SkipZeroBitStore* store);

/**
 * @brief 设置当前操作位置
 * @param store 存储结构指针
 * @param index 位置索引 (0-31)
 * @return 1表示设置成功，0表示失败
 */
uint8_t Labels_SetIndex(SkipZeroBitStore* store, uint8_t index);

/**
 * @brief 设置指定位置的位值为1
 * @param store 存储结构指针
 * @param index 位置索引 (0-31)
 * @return 1表示设置成功，0表示失败
 */
uint8_t Labels_SetBit(SkipZeroBitStore* store, uint8_t index);

/**
 * @brief 在当前位置设置位值为1，并移动到下一位
 * @param store 存储结构指针
 * @return 1表示设置成功，0表示失败
 */
uint8_t Labels_SetBitAndMove(SkipZeroBitStore* store);

/**
 * @brief 获取指定位置的位值
 * @param store 存储结构指针
 * @param index 位置索引 (0-31)
 * @param value 输出位值的指针
 * @return 1表示获取成功，0表示失败
 */
uint8_t Labels_GetBit(SkipZeroBitStore* store, uint8_t index, uint8_t* value);

/**
 * @brief 获取当前有效数据长度
 * @param store 存储结构指针
 * @return 有效数据长度
 */
uint8_t Labels_GetLength(SkipZeroBitStore* store);

/**
 * @brief 读取下一个位值(包括零值)
 * @param store 存储结构指针
 * @param value 输出位值的指针
 * @return 1表示读取成功，0表示已到达末尾
 */
uint8_t Labels_ReadNext(SkipZeroBitStore* store, uint8_t* value);

/**
 * @brief 清除所有位数据
 * @param store 存储结构指针
 */
void Labels_Clear(SkipZeroBitStore* store);

/**
 * @brief 设置多个连续位的值
 * @param store 存储结构指针
 * @param startIndex 开始位置
 * @param value 要设置的值
 * @param bitCount 位数
 * @return 1表示成功，0表示失败
 */
uint8_t Labels_SetBits(SkipZeroBitStore* store, uint8_t startIndex, uint32_t value, uint8_t bitCount);

/**
 * @brief 一次性获取多个连续位的值
 * @param store 存储结构指针
 * @param startIndex 开始位置
 * @param bitCount 位数 (最多32位)
 * @return 读取的值
 */
uint32_t Labels_GetBits(SkipZeroBitStore* store, uint8_t startIndex, uint8_t bitCount);

#endif