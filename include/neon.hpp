#include <arm_neon.h>
#include <stdio.h>
// neon 指令集网址：
// https://gcc.gnu.org/onlinedocs/gcc-4.7.4/gcc/ARM-NEON-Intrinsics.html#ARM-NEON-Intrinsics
uint8_t g_GammaLUT[256]; // 全局数组：包含256个元素的gamma校正查找表
inline void rbg24togray_neon(uint8_t *imgrgb, uint8_t *imggray, size_t nWidth, size_t nHeight)
{
    uint8_t *p_imgrgb = imgrgb;
    uint8_t *p_imggray = imggray;
    uint8x8_t r_corr = {76, 76, 76, 76, 76, 76, 76, 76};
    uint8x8_t g_corr = {150, 150, 150, 150, 150, 150, 150, 150};
    uint8x8_t b_corr = {30, 30, 30, 30, 30, 30, 30, 30};
    int16x8_t shift_r = {-8, -8, -8, -8, -8, -8, -8, -8};

    for (int i = 0; i < nHeight * nWidth; i++)
    {
        uint8x8x3_t vdata = vld3_u8(p_imgrgb);
        uint16x8_t vr = vmull_u8(r_corr, vdata.val[0]);
        uint16x8_t vg = vmlal_u8(vr, g_corr, vdata.val[1]);
        uint16x8_t vgray16 = vmlal_u8(vg, b_corr, vdata.val[2]);
        uint16x8_t vgray8 = vrshlq_u16(vgray16, shift_r);
        uint8x8_t vgray = vqmovn_u16(vgray8);
        vst1_u8(p_imggray, vgray);
        p_imggray++;
        p_imgrgb += 3;
    }
}
inline void resize_spilt_neon(uint8_t *img_1, uint8_t *img_2, const uint8_t *imgraw, size_t nWidth, size_t nHeight)
{
    if (NULL == img_1 || NULL == img_2 || NULL == imgraw)
    {
        return;
    }

    else
    {
        int height = 0;
        int width = 0;
        int width_2 = nWidth >> 1;
        int width_2_3 = 3 * width_2;
        int width_2_9 = 9 * width_2;
        while (height < nHeight)
        {
            while (width < width_2)
            {
                uint8x8x3_t vdata1 = vld3_u8(imgraw);
                uint8x8x3_t vdata2 = vld3_u8(imgraw + width_2_3);
                vst3_u8(img_1, vdata1);
                vst3_u8(img_2, vdata2);
                imgraw += 6;
                img_1 += 3;
                img_2 += 3;
                width += 2;
            }
            width = 0;
            imgraw += width_2_9;
            height += 2;
        }
        height = 0;
    }
}
inline void BuildTable(float fPrecompensation)
{
    int i;
    float f;
    for (i = 0; i < 256; i++)
    {
        f = (i + 0.5F) / 256;                      // 归一化
        f = (float)pow(f, fPrecompensation);       // 预补偿
        g_GammaLUT[i] = (uint8_t)(f * 256 - 0.5F); // 反归一化
    }
}

inline void GammaCorrectiom(uint8_t *src, int iWidth, int iHeight, uint8_t *Dst)
{
    int iCols, iRows;
    int pixelsNums = 0;
    // 对图像的每个像素进行查找表矫正
    for (iRows = 0; iRows < iHeight; iRows++)
    {
        for (iCols = 0; iCols < iWidth; iCols++)
        {
            int num = pixelsNums + iCols;
            Dst[num] = g_GammaLUT[src[num]];
        }
        pixelsNums += iWidth;
    }
}
// resize_spilt_neno(&img_right.data[0], &img_left.data[0], m_rgb_image, g_frame_data.nWidth, g_frame_data.nHeight);
// auto start = system_clock::now();
// rbg24togray_neon((&img_right.data[0]), (&img_right_gray.data[0]), 640, 512);
// auto end = system_clock::now();

// auto duration = duration_cast<microseconds>(end - start);
// std::cout << "time: " << double(duration.count()) * microseconds::period::num / microseconds::period::den << std::endl;
inline void spilt(uint8_t *img_1, uint8_t *img_2, const uint8_t *imgraw, size_t nWidth, size_t nHeight)
{
    if (NULL == img_1 || NULL == img_1 || NULL == imgraw)
    {
        return;
    }
    else
    {
        size_t height = 0;
        int nWidth_2_3 = nWidth / 2 * 3;
        int nWidth_2_6 = nWidth * 3;
        int length1 = 0;
        int length2 = 0;
        while (height < nHeight)
        {
            memcpy(img_1 + length1, imgraw + length2, nWidth_2_3);
            memcpy(img_2 + length1, imgraw + length2 + nWidth_2_3, nWidth_2_3);
            length1 += nWidth_2_3;
            length2 += nWidth_2_6;
            height++;
        }
    }
}
