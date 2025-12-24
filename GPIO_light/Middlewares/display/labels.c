#include "labels.h"


    
/**
 * @brief 计算有效数据长度
 * @param store 存储结构指针
 */
static void Labels_CalculateEffectiveLength(SkipZeroBitStore* store) {
    int8_t j;
    uint8_t found = 0;
    
    // 只需检查一个32位的buffer
    if (store->buffer == 0) {
        store->effectiveLength = 0;
        return;
    }
    
    // 找到最后一个非零位
    for (j = 31; j >= 0; j--) {
        if (store->buffer & (1UL << j)) {
            store->effectiveLength = j + 1;
            found = 1;
            break;
        }
    }
    
    // 如果没找到任何非零位，有效长度为0
    if (!found) {
        store->effectiveLength = 0;
    }
}

/**
 * @brief 初始化跳零位存储结构
 * @param store 存储结构指针
 */
void Labels_Init(SkipZeroBitStore* store) {
    // 清零缓冲区
    store->buffer = 0UL;
    
    store->currentIndex = 0;
    store->readIndex = 32;
    store->effectiveLength = 0;
}

/**
 * @brief 获取第一个非零值之前的零值数量
 * @param store 存储结构指针
 * @return 第一个非零值之前的零值数量
 */
uint8_t Labels_GetLeadingZeros(SkipZeroBitStore* store) {
    return store->readIndex;
}

/**
 * @brief 设置当前操作位置
 * @param store 存储结构指针
 * @param index 位置索引 (0-31)
 * @return 1表示设置成功，0表示失败
 */
uint8_t Labels_SetIndex(SkipZeroBitStore* store, uint8_t index) {
    if (index >= 32) {
        return 0;  // 超出范围
    }
    
    store->currentIndex = index;
    return 1;
}

/**
 * @brief 设置指定位置的位值为1
 * @param store 存储结构指针
 * @param index 位置索引 (0-31)
 * @return 1表示设置成功，0表示失败
 */
uint8_t Labels_SetBit(SkipZeroBitStore* store, uint8_t index) {
    if (index >= 32) {
        return 0;  // 超出范围
    }
    
    // 设置位为1
    store->buffer |= (1UL << index);  // 设置位
    
    // 更新第一个非零位位置
    if (store->readIndex > index) {
        store->readIndex = index;
    }
    
    // 更新有效长度
    if (index + 1 > store->effectiveLength) {
        store->effectiveLength = index + 1;
    }

    return 1;
}

/**
 * @brief 在当前位置设置位值为1，并移动到下一位
 * @param store 存储结构指针
 * @return 1表示设置成功，0表示失败
 */
uint8_t Labels_SetBitAndMove(SkipZeroBitStore* store) {
    uint8_t result;
    
    if (store->currentIndex >= 32) {
        return 0;  // 超出范围
    }
    
    result = Labels_SetBit(store, store->currentIndex);
    store->currentIndex = (store->currentIndex + 1) & 0x1F;  // 限制在0-31
    
    return result;
}

/**
 * @brief 获取指定位置的位值
 * @param store 存储结构指针
 * @param index 位置索引 (0-31)
 * @param value 输出位值的指针
 * @return 1表示获取成功，0表示失败
 */
uint8_t Labels_GetBit(SkipZeroBitStore* store, uint8_t index, uint8_t* value) {
    if (index >= 32) {
        return 0;  // 超出范围
    }
    
    // 读取位值
    *value = (store->buffer >> index) & 1UL;
    
    return 1;
}

/**
 * @brief 获取当前有效数据长度
 * @param store 存储结构指针
 * @return 有效数据长度
 */
uint8_t Labels_GetLength(SkipZeroBitStore* store) {
    return store->effectiveLength;
}

/**
 * @brief 读取下一个位值(包括零值)
 * @param store 存储结构指针
 * @param value 输出位值的指针
 * @return 1表示读取成功，0表示已到达末尾
 */
uint8_t Labels_ReadNext(SkipZeroBitStore* store, uint8_t* value) {
    // 如果已经读完所有有效数据，返回失败
    if (store->readIndex >= store->effectiveLength) {
        return 0;
    }
    
    // 读取当前位置的值
    Labels_GetBit(store, store->readIndex, value);
    store->readIndex++;
    
    return 1;
}

/**
 * @brief 清除所有位数据
 * @param store 存储结构指针
 */
void Labels_Clear(SkipZeroBitStore* store) {
    store->buffer = 0UL;
    
    store->currentIndex = 0;
    store->readIndex = 0;
    store->effectiveLength = 0;
}

/**
 * @brief 设置多个连续位的值
 * @param store 存储结构指针
 * @param startIndex 开始位置
 * @param value 要设置的值
 * @param bitCount 位数
 * @return 1表示成功，0表示失败
 */
uint8_t Labels_SetBits(SkipZeroBitStore* store, uint8_t startIndex, uint32_t value, uint8_t bitCount) {
    uint8_t i;
    
    // 确保不超出范围
    if (startIndex + bitCount > 32) {
        return 0;
    }
    
    // 限制位数最多32位
    if (bitCount > 32) {
        bitCount = 32;
    }
    
    // 逐位设置
    for (i = 0; i < bitCount; i++) {
        if ((value >> i) & 1UL) {
            Labels_SetBit(store, startIndex + i);
        }
    }
    
    return 1;
}

/**
 * @brief 一次性获取多个连续位的值
 * @param store 存储结构指针
 * @param startIndex 开始位置
 * @param bitCount 位数 (最多32位)
 * @return 读取的值
 */
uint32_t Labels_GetBits(SkipZeroBitStore* store, uint8_t startIndex, uint8_t bitCount) {
    uint32_t result = 0;
    uint8_t i, bitValue;
    
    // 限制位数最多32位
    if (bitCount > 32) {
        bitCount = 32;
    }
    
    // 确保不超出范围
    if (startIndex + bitCount > 32) {
        bitCount = 32 - startIndex;
    }
    
    // 逐位读取
    for (i = 0; i < bitCount; i++) {
        Labels_GetBit(store, startIndex + i, &bitValue);
        if (bitValue) {
            result |= (1UL << i);
        }
    }
    
    return result;
}