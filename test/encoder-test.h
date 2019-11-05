#include <mbed.h>
#include <Encoder.h>

Encoder mot1_encoder(ENCODER_1);
Encoder mot2_encoder(ENCODER_2);
Encoder mot3_encoder(ENCODER_3);
Encoder mot4_encoder(ENCODER_4);

int32_t count_mot1,count_mot2;

char checkDir(Encoder & enc, int32_t & cnt)
{
    int32_t new_count = enc.getCount();
    char dir =  new_count > cnt ? 'F' : (new_count < cnt ? 'R' : 'S');
    cnt = new_count;
    return dir; 
}

int test()
{
    printf("encoder-test: program started.\r\n");

    mot1_encoder.init();
    mot2_encoder.init();
    mot3_encoder.init();
    mot4_encoder.init();

    while(1)
    {
        printf("ENC1: %10d | ENC2: %10d | ENC3: %10d | ENC4: %10d\r\n",mot1_encoder.getCount(),mot2_encoder.getCount(),mot3_encoder.getCount(),mot4_encoder.getCount());
        ThisThread::sleep_for(50);
    }
}