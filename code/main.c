#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include "bsp.h"
#include "sysctl.h"
#include "plic.h"
#include "utils.h"
#include "gpiohs.h"
#include "fpioa.h"
#include "lcd.h"
#include "nt35310.h"
#include "dvp.h"
#include "ov5640.h"
#include "ov2640.h"
#include "uarths.h"
#include "kpu.h"
#include "region_layer.h"
#include "image_process.h"
#include "board_config.h"
#include "w25qxx.h"
#define INCBIN_STYLE INCBIN_STYLE_SNAKE
#define INCBIN_PREFIX
#include "incbin.h"
#include "utils.h"
#include "iomem.h"
#include "wifi.c"
#include "apple.c"
#include "wifioff.c"
#include "link.c"
#include "erweima.c"
#include "rtc.h"

#define PLL0_OUTPUT_FREQ 800000000UL
#define PLL1_OUTPUT_FREQ 400000000UL

volatile uint32_t g_ai_done_flag;
volatile uint8_t g_dvp_finish_flag;
static image_t kpu_image;
static uint32_t time_ram[8 * 16 * 8 / 2];
static uint32_t *g_lcd_gram0;
static uint32_t *g_lcd_gram1;
static uint32_t *mubiao;
volatile uint8_t g_ram_mux;
kpu_model_context_t face_detect_task;
static region_layer_t face_detect_rl;
static obj_info_t face_detect_info;
#define ANCHOR_NUM 5
static float anchor[ANCHOR_NUM * 2] = {1.16,1.56, 1.47,2.19, 2.08,2.85, 2.79,3.84, 4.21,4.63};
#define  LOAD_KMODEL_FROM_FLASH  1

#if LOAD_KMODEL_FROM_FLASH
#define KMODEL_SIZE (3100 * 1024)
uint8_t *model_data;
#else
INCBIN(model, "test.kmodel");
#endif

static void ai_done(void *ctx)
{
    g_ai_done_flag = 1;
}

static int on_irq_dvp(void* ctx)
{
    if (dvp_get_interrupt(DVP_STS_FRAME_FINISH))
    {
        /* switch gram */
        dvp_set_display_addr(g_ram_mux ? (uint32_t)g_lcd_gram0 : (uint32_t)g_lcd_gram1);

        dvp_clear_interrupt(DVP_STS_FRAME_FINISH);
        g_dvp_finish_flag = 1;
    }
    else
    {
        if(g_dvp_finish_flag == 0)
            dvp_start_convert();
        dvp_clear_interrupt(DVP_STS_FRAME_START);
    }

    return 0;
}

static void io_mux_init(void)
{

    /* Init DVP IO map and function settings */
    fpioa_set_function(42, FUNC_CMOS_RST);
    fpioa_set_function(44, FUNC_CMOS_PWDN);
    fpioa_set_function(46, FUNC_CMOS_XCLK);
    fpioa_set_function(43, FUNC_CMOS_VSYNC);
    fpioa_set_function(45, FUNC_CMOS_HREF);
    fpioa_set_function(47, FUNC_CMOS_PCLK);
    fpioa_set_function(41, FUNC_SCCB_SCLK);
    fpioa_set_function(40, FUNC_SCCB_SDA);

    /* Init SPI IO map and function settings */
    fpioa_set_function(38, FUNC_GPIOHS0 + DCX_GPIONUM);
    fpioa_set_function(36, FUNC_SPI0_SS3);
    fpioa_set_function(39, FUNC_SPI0_SCLK);
    fpioa_set_function(37, FUNC_GPIOHS0 + RST_GPIONUM);

    sysctl_set_spi0_dvp_data(1);

}
/*获取时间*/
void get_date_time()
{
    char time[25];
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    int w;
    rtc_timer_get(&year, &month, &day, &hour, &minute, &second);
    sprintf(time,"%02d:%02d:%02d", hour, minute, second);
    w=strlen(time)*8;
    printf("%s::%d\n",time,w );
    lcd_ram_draw_string(time,time_ram,BLACK ,WHITE );
    lcd_draw_picture(110,0, w, 16, time_ram);
}
/*时间中断*/
int on_timer_interrupt()
{
    get_date_time();
    return 0;
}



static void io_set_power(void)
{

        /* Set dvp and spi pin to 1.8V */
        sysctl_set_power_mode(SYSCTL_POWER_BANK6, SYSCTL_POWER_V18);
        sysctl_set_power_mode(SYSCTL_POWER_BANK7, SYSCTL_POWER_V18);

}

static void draw_edge(uint32_t *gram, obj_info_t *obj_info, uint32_t index, uint16_t color)
{
    uint32_t data = ((uint32_t)color << 16) | (uint32_t)color;
    uint32_t *addr1, *addr2, *addr3, *addr4, x1, y1, x2, y2;
    uint8_t x,y;
    uint16_t *addr5;
    x1 = obj_info->obj[index].x1;
    y1 = obj_info->obj[index].y1;
    x2 = obj_info->obj[index].x2;
    y2 = obj_info->obj[index].y2;

    if (x1 <= 0)
        x1 = 1;
    if (x2 >= 223)
        x2 = 223;
    if (y1 <= 0)
        y1 = 1;
    if (y2 >= 223)
        y2 = 223;
    //lcd_draw_string(x1,y1, "apple", GREEN);
    addr1 = gram + (224 * y1 + x1) / 2;
    addr2 = gram + (224 * y1 + x2 - 8) / 2;
    addr3 = gram + (224 * (y2 - 1) + x1) / 2;
    addr4 = gram + (224 * (y2 - 1) + x2 - 8) / 2;
    for (uint32_t i = 0; i < 4; i++)
    {
        *addr1 = data;
        *(addr1 + 112) = data;
        *addr2 = data;
        *(addr2 + 112) = data;
        *addr3 = data;
        *(addr3 + 112) = data;
        *addr4 = data;
        *(addr4 + 112) = data;
        addr1++;
        addr2++;
        addr3++;
        addr4++;
    }
    addr1 = gram + (224 * y1 + x1) / 2;
    addr2 = gram + (224 * y1 + x2 - 2) / 2;
    addr3 = gram + (224 * (y2 - 8) + x1) / 2;
    addr4 = gram + (224 * (y2 - 8) + x2 - 2) / 2;
    for (uint32_t i = 0; i < 8; i++)
    {
        *addr1 = data;
        *addr2 = data;
        *addr3 = data;
        *addr4 = data;
        addr1 += 112;
        addr2 += 112;
        addr3 += 112;
        addr4 += 112;
    }
/*画中心区域*/
    addr1 = gram + (224 * 112 + 100) / 2;
    for (uint32_t i = 0; i < 13; i++)
    {
        *addr1 = data;
        *(addr1 + 112) = data;
        addr1 ++;

    }
       addr1 = gram + (224 * 100 + 112) / 2;
    for (uint32_t i = 0; i < 25; i++)
    {
        *addr1 = data;   
         addr1 += 112;
       
    }
    /*画苹果中心*/
    x=(x1+x2)/2;
    y=(y1+y2)/2;
 addr5 = gram + (224 * (y-3)+ (x-3)) / 2;
    for (uint32_t i = 0; i < 7; i++)
    {
        *addr5 = data; 
        *(addr5 + 1) = data;
         addr5 += 225;
       
    }
    addr5 = gram + (224 * (y-3)+ (x+3)) / 2;
    for (uint32_t i = 0; i < 7; i++)
    {
        *addr5 = data;  
        *(addr5 + 1) = data; 
         addr5 += 223;
       
    }

}

int main(void)
{
    /* Set CPU and dvp clk */
    sysctl_pll_set_freq(SYSCTL_PLL0, PLL0_OUTPUT_FREQ);
    sysctl_pll_set_freq(SYSCTL_PLL1, PLL1_OUTPUT_FREQ);
    sysctl_clock_enable(SYSCTL_CLOCK_AI);
    uarths_init();
    io_set_power();
    io_mux_init();
    plic_init();
     rtc_init();
    /* flash init */
    printf("flash init\n");
    w25qxx_init(3, 0);
    w25qxx_enable_quad_mode();
#if LOAD_KMODEL_FROM_FLASH
    model_data = (uint8_t *)malloc(KMODEL_SIZE + 255);
    uint8_t *model_data_align = (uint8_t *)(((uintptr_t)model_data+255)&(~255));
    w25qxx_read_data(0x300000, model_data_align, KMODEL_SIZE, W25QXX_QUAD_FAST);
#else
    uint8_t *model_data_align = model_data;
#endif


    /* LCD init */
    printf("LCD init\n");
    lcd_init();
    lcd_set_direction(DIR_YX_RLDU);

    lcd_clear(WHITE);
    //lcd_draw_ico(0,0,16,16,gImage_wifi,BLACK);
    lcd_draw_ico(0,0,16,16,gImage_wifioff,BLACK);
   // lcd_draw_ico(240,40,64,64,gImage_link,BLACK);
    lcd_draw_ico(240,16,64,64,gImage_apple,RED);
    //lcd_draw_ico(30,30,192,192,gImage_erweima,RED);
    
    g_lcd_gram0 = (uint32_t *)iomem_malloc(224*224*2);
    g_lcd_gram1 = (uint32_t *)iomem_malloc(224*224*2);



    /* DVP init */
    printf("DVP init\n");

    dvp_init(8);
    dvp_set_xclk_rate(24000000);
    dvp_enable_burst();
    dvp_set_output_enable(0, 1);
    dvp_set_output_enable(1, 1);
    dvp_set_image_format(DVP_CFG_RGB_FORMAT);
    dvp_set_image_size(224, 224);
    ov2640_init();



    kpu_image.pixel = 3;
    kpu_image.width = 224;
    kpu_image.height = 224;
    image_init(&kpu_image);

    dvp_set_ai_addr((uint32_t)kpu_image.addr, (uint32_t)(kpu_image.addr + 224 * 224), (uint32_t)(kpu_image.addr + 224 * 224 * 2));
    dvp_set_display_addr((uint32_t)g_lcd_gram0);
    dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 0);
    dvp_disable_auto();
    /* DVP interrupt config */
    printf("DVP interrupt config\n");
    plic_set_priority(IRQN_DVP_INTERRUPT, 1);
    plic_irq_register(IRQN_DVP_INTERRUPT, on_irq_dvp, NULL);
    plic_irq_enable(IRQN_DVP_INTERRUPT);
    /* init face detect model */
    if (kpu_load_kmodel(&face_detect_task, model_data_align) != 0)
    {
        printf("\nmodel init error\n");
        while (1);
    }
    face_detect_rl.anchor_number = ANCHOR_NUM;
    face_detect_rl.anchor = anchor;
    face_detect_rl.threshold = 0.7;
    face_detect_rl.nms_value = 0.3;
    region_layer_init(&face_detect_rl, 7, 7, 30, kpu_image.width, kpu_image.height);
    /* enable global interrupt */




    sysctl_enable_irq();
    /* system start */
    printf("System start\n");
    dvp_clear_interrupt(DVP_STS_FRAME_START | DVP_STS_FRAME_FINISH);
    dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 1);
    uint64_t time_last = sysctl_get_time_us();
    uint64_t time_now = sysctl_get_time_us();
    int time_count = 0;
    int fs=0;
    char sfs[15];
    /*时间配置*/
    rtc_timer_set(2018, 9, 12, 23, 30, 50);
    //rtc_alarm_set(2018, 9, 12, 23, 31, 00);

    printf("RTC Tick and Alarm Test\n" "Compiled in " __DATE__ " " __TIME__ "\n");

    // rtc_tick_irq_register(
    //     false,
    //     RTC_INT_SECOND,
    //     on_timer_interrupt,
    //     NULL,
    //     1
    // );
    uint8_t w,h,mx,my,an_count=0,ay_count=0,app_exist=0,app_disappear=0;
    uint32_t area,app_count=0;
    while (1)
    {
        
        
        while (g_dvp_finish_flag == 0){

        for (uint32_t face_cnt = 0; face_cnt < face_detect_info.obj_number; face_cnt++)
        {
            an_count=0;
            draw_edge((uint32_t *)g_ram_mux ? g_lcd_gram0 : g_lcd_gram1, &face_detect_info, face_cnt, RED);
         

             if(face_detect_info.obj[face_cnt].x2>223)
                face_detect_info.obj[face_cnt].x2=223;

             if(face_detect_info.obj[face_cnt].y2>223)
             {
                 face_detect_info.obj[face_cnt].y2=223;
             }
               
                w = face_detect_info.obj[face_cnt].x2-face_detect_info.obj[face_cnt].x1;
                h = face_detect_info.obj[face_cnt].y2-face_detect_info.obj[face_cnt].y1;
                /*计算中点*/
                mx = (face_detect_info.obj[face_cnt].x2+face_detect_info.obj[face_cnt].x1)/2;
                my= (face_detect_info.obj[face_cnt].y2+face_detect_info.obj[face_cnt].y1)/2;

                sprintf(sfs,"W:%3d h:%3d", w,h);
                lcd_ram_draw_string(sfs,time_ram,BLACK ,WHITE );
                lcd_draw_picture(230,90+32*face_cnt, strlen(sfs)*8, 16, time_ram);
                /*计算面积*/
                 area=h*w;
                sprintf(sfs,"area:%d", area);
              
                lcd_ram_draw_string(sfs,time_ram,BLACK ,WHITE );
                lcd_draw_picture(230,106+32*face_cnt, strlen(sfs)*8, 16, time_ram);
                /*苹果计数*/
                if(abs(mx-112)<12&&abs(my-112)<12)
                {
                    app_disappear=0;
                    ay_count++;
                    if(ay_count==3&&app_exist==0)//检测到苹果
                    {
                        sysctl_disable_irq();//禁用系统中断
                        app_count++;
                        app_exist=1;
                        lcd_clear_xy(0,16,223,239,WHITE);
                        mubiao = (uint32_t *)iomem_malloc((w+1)*(h+1)*2);//申请空间
                        

                        /*for(uint8_t i=face_detect_info.obj[face_cnt].y1;i<=face_detect_info.obj[face_cnt].y2;i++)
                        {

                           lcd_draw_picture(face_detect_info.obj[face_cnt].x1,i+15, w+1, 1, 
                            (uint32_t *)(g_ram_mux ? g_lcd_gram0 : g_lcd_gram1)+(224*i+face_detect_info.obj[face_cnt].x1)/2);
                

                        }*/
                        for(uint8_t i=0;i<=h;i++)
                        {
                        dmac_wait_idle ( SYSCTL_DMA_CHANNEL_1 );//选择通道一
                        dmac_set_single_mode ( SYSCTL_DMA_CHANNEL_1 , (uint32_t *)(g_ram_mux ? g_lcd_gram0 : g_lcd_gram1)
                            +(224*(face_detect_info.obj[face_cnt].y1+i)+face_detect_info.obj[face_cnt].x1)/2 ,
                        mubiao+(w+1)*i/2,
DMAC_ADDR_INCREMENT , DMAC_ADDR_INCREMENT , DMAC_MSIZE_4 , DMAC_TRANS_WIDTH_32 , (w+1)/2);


                        }

                        lcd_draw_picture(face_detect_info.obj[face_cnt].x1,face_detect_info.obj[face_cnt].y1+16, w+1, h+1, 
                            mubiao);

                         iomem_free(mubiao);
                       msleep(500);
                        sysctl_enable_irq();//开启中断
                    }

                }
                else
                {
                    app_disappear++;
                    if(app_disappear==3){
                    ay_count=0;
                    app_exist=0;

                    }
                  
                }
                sprintf(sfs,"number:%4d", app_count);
              
                lcd_ram_draw_string(sfs,time_ram,BLACK ,WHITE );
                lcd_draw_picture(230,220, strlen(sfs)*8, 16, time_ram);

        }
        /*检测十次没有苹果清屏*/
        if(!face_detect_info.obj_number)
        {
            an_count++;
            if(an_count==10)
            lcd_clear_xy(230,90,320,200,WHITE);

        }
        /* display result */
        lcd_draw_picture(0, 16, 224, 224, g_ram_mux ? g_lcd_gram0 : g_lcd_gram1);
        /*计算帧率*/
       time_count ++;
        if(time_count % 10 == 0)
        {
            time_now = sysctl_get_time_us();
            fs=(int)10000000/(time_now - time_last);
            //itoa(fs,sfs,10);
            sprintf(sfs,"FS:%d", fs);
            //lcd_draw_string(240,0,sfs,RED);
            printf("%s\n",sfs );
            time_last = time_now;
             get_date_time();//刷新时间

            lcd_ram_draw_string(sfs,time_ram,BLACK ,WHITE );
            lcd_draw_picture(280,0, strlen(sfs)*8, 16, time_ram);

        }
        while (g_dvp_finish_flag == 0);
        }
            ;
        g_dvp_finish_flag = 0;
        g_ram_mux ^= 0x01;

        /* run face detect */
        g_ai_done_flag = 0;
        kpu_run_kmodel(&face_detect_task, kpu_image.addr, DMAC_CHANNEL5, ai_done, NULL);
        while(!g_ai_done_flag);
       
        float *output;
        size_t output_size;
        kpu_get_output(&face_detect_task, 0, (uint8_t **)&output, &output_size);
        face_detect_rl.input = output;
        region_layer_run(&face_detect_rl, &face_detect_info);
        /* run key point detect */
  
    }
}
