#include "render.h"


SkipZeroBitStore signalLabels[MAX_ROWS][4];  // 存储帧变化标记
u8 frameBuffer[MAX_ROWS][MAX_COLUMNS];    // 显示缓冲区
RenderState state;       // 渲染状态

int abs(int x) {
    return (x < 0) ? -x : x;
}

void Write_Prepare(void) {
    /* Wait until I2C bus is ready */
    while (I2C_GetFlagStatus(TARGET_I2C, I2C_FLAG_BUSY));
    /* Send start bit */
    I2C_GenerateSTART(TARGET_I2C, ENABLE);
    while (!I2C_CheckEvent(TARGET_I2C, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(TARGET_I2C, 0x78, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(TARGET_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    state = RENDER_DRAWING;  // 设置状态为绘制
}

void Write_Complete(void) {
    /* Send stop bit */
    I2C_GenerateSTOP(TARGET_I2C, ENABLE);

}

void Write_Position(uint8_t row, uint8_t column) {
    column+=2;
    /* Send loc byte */
    Write_Prepare();
    I2C_SendData(TARGET_I2C, 0x00);
    while (!I2C_CheckEvent(TARGET_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_SendData(TARGET_I2C, 0xB0 + row);
    while (!I2C_CheckEvent(TARGET_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    Write_Complete();

    Write_Prepare();
    I2C_SendData(TARGET_I2C, 0x00);
    while (!I2C_CheckEvent(TARGET_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_SendData(TARGET_I2C, ((column & 0xF0) >> 4) | 0x10);
    while (!I2C_CheckEvent(TARGET_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    Write_Complete();

    Write_Prepare();
    I2C_SendData(TARGET_I2C, 0x00);
    while (!I2C_CheckEvent(TARGET_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_SendData(TARGET_I2C, (column & 0x0F) | 0x01);
    while (!I2C_CheckEvent(TARGET_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    Write_Complete();
}

void Write_Byte(uint8_t dat) {
    /* Send pixel byte */
    Write_Prepare();
    I2C_SendData(TARGET_I2C, 0x40);
    while (!I2C_CheckEvent(TARGET_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_SendData(TARGET_I2C, dat);
    while (!I2C_CheckEvent(TARGET_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    Write_Complete();
}




void Render_Init(void) {
    // for (uint8_t i = 0; i < MAX_ROWS; i++) {
    //     for (uint8_t j = 0; j < 4; j++) {
    //         Labels_Init(&signalLabels[i][j]);  // 初始化所有四列的跳零位存储结构
    //     }
    // }
    // for (uint8_t i = 0; i < MAX_ROWS; i++) {
    //     for (uint8_t j = 0; j < MAX_COLUMNS; j++) {
    //         frameBuffer[i][j] = 0x00;  // 初始化显示缓冲区
    //     }
    // }
    state = RENDER_IDLE;  // 初始化状态为空闲
}

void Render_Test(void) {

    // Draw vertical lines with one space in between
    // for (uint8_t x = 0; x < 127; x += 2) {
    //     Draw_Line(x, 0, x, 63, DRAW_ON);  // Draw vertical line from top to bottom
    // }

    Draw_Line(126, 0, 0, 63, LINE_SOLID, DRAW_ON);  // 绘制线条
    Draw_Line(0, 0, 126, 63, LINE_SOLID, DRAW_ON);  // 绘制线条

    Draw_Line(0, 0, 63, 63, LINE_SOLID, DRAW_ON);  // 绘制线条
    Draw_Line(63, 63, 126, 0, LINE_SOLID, DRAW_ON);  // 绘制线条
    Draw_Line(0, 63, 63, 0, LINE_SOLID, DRAW_ON);  // 绘制线条
    Draw_Line(63, 0, 126, 63, LINE_SOLID, DRAW_ON);  // 绘制线条

    // uint8_t c = 0, i = 0;
    // c = 'E' - ' ';  // 得到偏移后的值
    // for (i = 0; i < 6; i++) {
    //     Draw_Byte(i, 3, F6x8[c][i], DRAW_ON);
    // }

    // Draw_Rectangle(0, 0, 126, 63, LINE_SOLID, FILL_NONE, DRAW_ON);  // 绘制矩形

    Draw_Arc(63, 31, 21, QUAD_TOP_LEFT, LINE_SOLID, DRAW_ON);  // 绘制四分之一圆弧
    Draw_Arc(63, 31, 21, QUAD_BOTTOM_LEFT, LINE_DASHED, DRAW_ON);  // 绘制四分之一圆弧
    Draw_Arc(63, 31, 21, QUAD_TOP_RIGHT, LINE_SOLID, DRAW_ON);  // 绘制四分之一圆弧
    Draw_Arc(63, 31, 21, QUAD_BOTTOM_RIGHT, LINE_DASHED, DRAW_ON);  // 绘制四分之一圆弧

}

void Render_FrameBuffer(void) {
    if (state != RENDER_IDLE) return;  // 如果不是空闲状态，返回

    state = RENDER_DRAWING;  // 设置状态为绘制

    for (uint8_t row_index = 0; row_index < MAX_ROWS; row_index++) {
        for (uint8_t col_group = 0; col_group < 4; col_group++) {
            uint8_t group_length = Labels_GetLength(&signalLabels[row_index][col_group]);
            if (group_length > 0) {
                uint8_t leadingZeros = Labels_GetLeadingZeros(&signalLabels[row_index][col_group]);
                uint8_t bitCount = group_length - leadingZeros;  // 有效位数
                uint8_t base_column = col_group * 32;  // 每列对应32个像素
                uint8_t column_index = base_column + leadingZeros;
                Write_Position(row_index, column_index);
                for (uint8_t i = 0; i < bitCount; i++) {
                    Write_Byte(frameBuffer[row_index][column_index]);
                    column_index++;
                }
                Labels_Clear(&signalLabels[row_index][col_group]);  // 清除当前块的标记
            }
        }
    }

    state = RENDER_IDLE;  // 设置状态为空闲
}


void Draw_Byte(uint8_t x, uint8_t y, uint8_t dat, DrawType type) {
    uint8_t row = y >> 3;  // 行号
    uint8_t line_offset = y & 0x07;  // 行内偏移
    if (row >= MAX_ROWS || x >= MAX_COLUMNS) return;  // 检查坐标是否超出范围
    
    uint8_t col_group = x >> 5;  // 除以32，确定是哪一组
    uint8_t col_offset = x & 0x1F;  // 在组内的偏移（0-31）
    
    switch (type) {
    case DRAW_ON:
        frameBuffer[row][x] |= (dat << line_offset);
        break;
    case DRAW_OFF:
        frameBuffer[row][x] &= ~(dat << line_offset);
        break;
    case DRAW_SWITCH:
        frameBuffer[row][x] ^= (dat << line_offset);
        break;
    default:
        frameBuffer[row][x] = (dat << line_offset);
        break;
    }
    
    if (line_offset > 0 && row + 1 < MAX_ROWS) {
        switch (type) {
        case DRAW_ON:
            frameBuffer[row + 1][x] |= (dat >> (8 - line_offset));
            break;
        case DRAW_OFF:
            frameBuffer[row + 1][x] &= ~(dat >> (8 - line_offset));
            break;
        case DRAW_SWITCH:
            frameBuffer[row + 1][x] ^= (dat >> (8 - line_offset));
            break;
        default:
            frameBuffer[row + 1][x] = (dat >> (8 - line_offset));
            break;
        }
        Labels_SetBit(&signalLabels[row + 1][col_group], col_offset);  // 设置下一行的标记
    }
    
    Labels_SetBit(&signalLabels[row][col_group], col_offset);  // 设置当前行的标记
}

void Draw_Pixel(uint8_t x, uint8_t y, DrawType type) {
    uint8_t row = y >> 3;
    uint8_t line_offset = y & 0x07;
    if (row >= MAX_ROWS || x >= MAX_COLUMNS) return;  // 检查坐标是否超出范围
    
    uint8_t col_group = x >> 5;  // 除以32，确定是哪一组
    uint8_t col_offset = x & 0x1F;  // 在组内的偏移（0-31）
    
    switch (type) {
    case DRAW_ON:
        frameBuffer[row][x] |= (1 << line_offset);  // 设置对应位为1
        break;
    case DRAW_OFF:
        frameBuffer[row][x] &= ~(1 << line_offset);  // 设置对应位为0
        break;
    case DRAW_SWITCH:
        frameBuffer[row][x] ^= (1 << line_offset);  // 切换对应位
        break;
    default:
        break;
    }
    Labels_SetBit(&signalLabels[row][col_group], col_offset);  // 设置标记
}

void Draw_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, LineType line, DrawType type) {
    // Bresenham's line algorithm
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;
    int pixelCount = 0; // Counter for dashed line pattern
    
    while (x1 != x2 || y1 != y2) {
        // For dashed lines, only draw every other few pixels
        if (line == LINE_SOLID || (line == LINE_DASHED && (pixelCount % 6) < 3)) {
            Draw_Pixel(x1, y1, type);  // 绘制像素点
        }
        pixelCount++;
        
        int err2 = err * 2;
        if (err2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (err2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
    
    // Always draw the last pixel regardless of pattern
    Draw_Pixel(x2, y2, type);
}

void Draw_Rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, LineType line, FillType fillType, DrawType type) {
    // Draw the rectangle edges without redrawing corners
    Draw_Line(x1, y1, x2 - 1, y1, line, type);  // Top line (excludes top-right corner)
    Draw_Line(x2, y1, x2, y2 - 1, line, type);  // Right line (excludes bottom-right corner)
    Draw_Line(x2, y2, x1 + 1, y2, line, type);  // Bottom line (excludes bottom-left corner)
    Draw_Line(x1, y2, x1, y1 + 1, line, type);  // Left line (excludes top-left corner)

    // Fill the rectangle if required
    if (fillType != FILL_NONE) {
        uint8_t pattern = (fillType == FILL_SOLID) ? 0xFF : 0xAA; // Solid or pattern fill
        
        // Fill the inside of the rectangle (excluding the border)
        for (uint8_t y = y1 + 1; y < y2; y++) {
            for (uint8_t x = x1 + 1; x < x2; x++) {
                // For pattern fill, alternate the pattern on each row
                uint8_t currentPattern = (fillType == FILL_PATTERN && (y % 2)) ? ~pattern : pattern;
                // For pattern fill, only set pixels based on column pattern
                if (fillType == FILL_SOLID || (x % 2 == 0 && (currentPattern & 0xAA)) || 
                                             (x % 2 == 1 && (currentPattern & 0x55))) {
                    Draw_Pixel(x, y, type);
                }
            }
        }
    }
}

void Draw_RoundedRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t r, LineType line, FillType fillType, DrawType type) {
    // Draw the rectangle edges with rounded corners
    Draw_Line(x1 + r, y1, x2 - r, y1, line, type);  // Top line (excludes top-right corner)
    Draw_Line(x1 + r, y2, x2 - r, y2, line, type);  // Bottom line (excludes bottom-left corner)
    Draw_Line(x1, y1 + r, x1, y2 - r, line, type);  // Left line (excludes top-left corner)
    Draw_Line(x2, y1 + r, x2, y2 - r, line, type);  // Right line (excludes top-right corner)

    // Draw the rounded corners
    Draw_Arc(x1 + r, y1 + r, r, QUAD_TOP_LEFT, line, type);  // Top-left corner
    Draw_Arc(x2 - r, y1 + r, r, QUAD_TOP_RIGHT, line, type);  // Top-right corner
    Draw_Arc(x1 + r, y2 - r, r, QUAD_BOTTOM_LEFT, line, type);  // Bottom-left corner
    Draw_Arc(x2 - r, y2 - r, r, QUAD_BOTTOM_RIGHT, line, type);  // Bottom-right corner

    // Fill the rectangle if required
    if (fillType != FILL_NONE) {
        uint8_t pattern = (fillType == FILL_SOLID) ? 0xFF : 0xAA; // Solid or pattern fill
        
        // Fill the inside of the rectangle (excluding the border)
        for (uint8_t y = y1 + 1; y < y2; y++) {
            for (uint8_t x = x1 + 1; x < x2; x++) {
                // Skip pixels in the corner regions that are outside the rounded rectangle
                if ((y < y1 + r && x < x1 + r && (x - (x1 + r))*(x - (x1 + r)) + (y - (y1 + r))*(y - (y1 + r)) > r*r) || // Top-left corner
                    (y < y1 + r && x > x2 - r && (x - (x2 - r))*(x - (x2 - r)) + (y - (y1 + r))*(y - (y1 + r)) > r*r) || // Top-right corner
                    (y > y2 - r && x < x1 + r && (x - (x1 + r))*(x - (x1 + r)) + (y - (y2 - r))*(y - (y2 - r)) > r*r) || // Bottom-left corner
                    (y > y2 - r && x > x2 - r && (x - (x2 - r))*(x - (x2 - r)) + (y - (y2 - r))*(y - (y2 - r)) > r*r))   // Bottom-right corner
                    continue;
                
                // For pattern fill, alternate the pattern on each row
                uint8_t currentPattern = (fillType == FILL_PATTERN && (y % 2)) ? ~pattern : pattern;
                // For pattern fill, only set pixels based on column pattern
                if (fillType == FILL_SOLID || (x % 2 == 0 && (currentPattern & 0xAA)) || 
                                             (x % 2 == 1 && (currentPattern & 0x55))) {
                    Draw_Pixel(x, y, type);
                }
            }
        }
    }
}

void Draw_Arc(uint8_t x, uint8_t y, uint8_t r, QuadrantType quadrant, LineType line, DrawType type) {
    // Draw the arc using Bresenham's circle algorithm (quarter circle)
    int dx = r;
    int dy = 0;
    int err = 0;
    int pixelCount = 0; // Counter for dashed line pattern
    
    while (dx >= dy) {
        // Only draw if LINE_SOLID or if it's part of the dash pattern
        if (line == LINE_SOLID || (line == LINE_DASHED && (pixelCount % 6) < 3)) {
            int x1, y1, x2, y2;
            
            // Calculate pixel positions based on the octant
            switch(quadrant) {
                case QUAD_TOP_LEFT:
                    x1 = x - dx; y1 = y - dy;
                    x2 = x - dy; y2 = y - dx;
                    break;
                case QUAD_BOTTOM_LEFT:
                    x1 = x - dx; y1 = y + dy;
                    x2 = x - dy; y2 = y + dx;
                    break;
                case QUAD_TOP_RIGHT:
                    x1 = x + dx; y1 = y - dy;
                    x2 = x + dy; y2 = y - dx;
                    break;
                case QUAD_BOTTOM_RIGHT:
                    x1 = x + dx; y1 = y + dy;
                    x2 = x + dy; y2 = y + dx;
                    break;
            }
            
            // Adjust endpoints for proper arc appearance
            if (dy == 0 && dx == r) {
                // Adjust horizontal endpoint (subtract 1 from outer position)
                x1 += (quadrant == QUAD_TOP_LEFT || quadrant == QUAD_BOTTOM_LEFT) ? 1 : -1;
            } else if (dx == 0 && dy == r) {
                // Adjust vertical endpoint
                y2 += (quadrant == QUAD_TOP_LEFT || quadrant == QUAD_TOP_RIGHT) ? 1 : -1;
            }
            
            // Draw the points
            Draw_Pixel(x1, y1, type);
            if (dx != dy) { // Avoid duplicate pixels at 45-degree points
                Draw_Pixel(x2, y2, type);
            }
        }
        
        pixelCount++;
        
        // Moving to the next pixel
        if (2 * (err + dy) + 1 < 2 * (1 - dx)) {
            dy++;
            err += 2*dy + 1;
        } else {
            dy++;
            dx--;
            err += 2*(dy - dx) + 2;
        }
    }
}

void Draw_Quadrant(uint8_t x, uint8_t y, uint8_t r, QuadrantType quadrant, LineType line, FillType fill, DrawType type) {
    // Draw the arc for the specified quadrant
    Draw_Arc(x, y, r, quadrant, line, type);
    
    // Draw the straight border lines based on quadrant
    switch(quadrant) {
        case QUAD_TOP_LEFT:
            Draw_Line(x, y, x, y-r, line, type);  // Vertical line
            Draw_Line(x, y, x-r, y, line, type);  // Horizontal line
            break;
        case QUAD_BOTTOM_LEFT:
            Draw_Line(x, y, x, y+r, line, type);  // Vertical line
            Draw_Line(x, y, x-r, y, line, type);  // Horizontal line
            break;
        case QUAD_TOP_RIGHT:
            Draw_Line(x, y, x, y-r, line, type);  // Vertical line
            Draw_Line(x, y, x+r, y, line, type);  // Horizontal line
            break;
        case QUAD_BOTTOM_RIGHT:
            Draw_Line(x, y, x, y+r, line, type);  // Vertical line
            Draw_Line(x, y, x+r, y, line, type);  // Horizontal line
            break;
    }
    
    // Fill the quadrant if required
    if (fill != FILL_NONE) {
        // Calculate fill area boundaries based on quadrant
        int x_start, x_end, y_start, y_end;
        
        switch(quadrant) {
            case QUAD_TOP_LEFT:
                x_start = x-r+1; x_end = x-1; y_start = y-r+1; y_end = y-1;
                break;
            case QUAD_BOTTOM_LEFT:
                x_start = x-r+1; x_end = x-1; y_start = y+1; y_end = y+r-1;
                break;
            case QUAD_TOP_RIGHT:
                x_start = x+1; x_end = x+r-1; y_start = y-r+1; y_end = y-1;
                break;
            case QUAD_BOTTOM_RIGHT:
                x_start = x+1; x_end = x+r-1; y_start = y+1; y_end = y+r-1;
                break;
            default:
                return;
        }
        
        uint8_t pattern = (fill == FILL_SOLID) ? 0xFF : 0xAA; // Solid or pattern fill
        
        // Fill the inside of the quadrant (excluding the borders)
        for (int py = y_start; py <= y_end; py++) {
            for (int px = x_start; px <= x_end; px++) {
                // Skip pixels outside the arc radius
                int dx = px - x;
                int dy = py - y;
                if (dx*dx + dy*dy > r*r) continue;
                
                // For pattern fill, alternate the pattern on each row
                uint8_t currentPattern = (fill == FILL_PATTERN && (py % 2)) ? ~pattern : pattern;
                if (fill == FILL_SOLID || (px % 2 == 0 && (currentPattern & 0xAA)) || 
                                         (px % 2 == 1 && (currentPattern & 0x55))) {
                    Draw_Pixel(px, py, type);
                }
            }
        }
    }
}




void I2C_Error_Handler(void) {
    /* Handle I2C errors by resetting the communication */
    
    /* Generate STOP condition to release the bus */
    I2C_GenerateSTOP(TARGET_I2C, ENABLE);
    
    /* Clear error flags */
    I2C_ClearFlag(TARGET_I2C, I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
    
    /* Reset I2C peripheral if needed */
    I2C_SoftwareResetCmd(TARGET_I2C, ENABLE);
    I2C_SoftwareResetCmd(TARGET_I2C, DISABLE);
    
    /* Reset state to idle */
    state = RENDER_ERROR;
}

