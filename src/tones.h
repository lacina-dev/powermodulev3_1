#include <Arduino.h>

#define _PWM_LOGLEVEL_       0

#include "AVR_PWM.h"

#define BUZZER       45  // Buzzer pin


// Note tone
#define T00  0.0f  // No tone 
#define TC0  16.0f
#define TD0  18.0f
#define TE0  20.0f
#define TF0  22.0f
#define TG0  25.0f
#define TA0  28.0f
#define TB0  31.0f
#define TC1  33.0f
#define TD1  37.0f
#define TE1  41.0f
#define TF1  44.0f
#define TG1  49.0f
#define TA1  55.0f
#define TB1  62.0f
#define TC2  66.0f
#define TD2  73.0f
#define TE2  82.0f
#define TF2  87.0f
#define TG2  98.0f
#define TA2  110.0f
#define TB2  123.0f
#define TC3  131.0f
#define TD3  147.0f
#define TE3  165.0f
#define TF3  175.0f
#define TG3  196.0f
#define TA3  220.0f
#define TB3  247.0f
#define TC4  262.0f
#define TD4  294.0f
#define TE4  330.0f
#define TF4  349.0f
#define TG4  392.0f
#define TA4  440.0f
#define TB4  493.0f
#define TC5  523.0f
#define TD5  587.0f
#define TE5  659.0f
#define TF5  698.0f
#define TG5  783.0f
#define TA5  880.0f
#define TB5  987.0f
#define TC6  1046.0f
#define TD6  1174.0f
#define TE6  1318.0f
#define TF6  1396.0f
#define TG6  1567.0f
#define TA6  1760.0f
#define TB6  1975.0f
#define TC7  2093.0f
#define TD7  2349.0f
#define TE7  2637.0f
#define TF7  2793.0f
#define TG7  3135.0f
#define TA7  3520.0f
#define TB7  3951.0f
#define TC8  4186.0f
#define TD8  4698.0f
#define TE8  5274.0f
#define TF8  5587.0f
#define TG8  6271.0f
#define TA8  7040.0f
#define TB8  7902.0f

// Tone length
#define L01  2000  // 1/1 in [ms]
#define L01  2000  // 1/1 in [ms]
#define L02  1000  // 1/2 
#define L04  500  // 1/4 
#define L08  250   // 1/8 
#define L16  125   // 1/16
#define L32  63   // 1/32
#define L64  31   // 1/64
#define L00  0  // zero

// Volume
#define VHI  90.0f  // High loud
#define VME  3.0f  // Medium loud
#define VLO  1.0f  // Low loud
#define VNO  0.0f  // No loud


struct melodya{
   int melody_name;
   int id;
   int note_count; // 
   float note[7];
   float note_length[7];
   float note_volume[7];
};

struct melodya melody_online = { 
    101,  // melody_id
    100, // id to start
    7, // num of notes
   {TC6, T00, TD6, T00, TG6, T00, T00},  // tones
   {L32, L32, L32, L32, L32, L00, L00},  // length
   {VHI, VNO, VHI, VNO, VHI, VNO, VNO}   // volume
};

struct melodya melody_offline = { 
    102,  // melody_id
    100, // id to start
    7, // num of notes
   {TG6, T00, TF6, T00, TC6, T00, T00},  // tones
   {L32, L32, L32, L32, L32, L00, L00},  // length
   {VHI, VNO, VHI, VNO, VHI, VNO, VNO}   // volume
};

struct melodya melody_charging = { 
    103,  // melody_id
    100, // id to start
    7, // num of notes
   {T00, TF8, T00, TG8, T00, T00, T00},  // tones
   {L01, L32, L32, L32, L00, L00, L00},  // length
   {VNO, VHI, VNO, VHI, VNO, VNO, VNO}   // volume
};

struct melodya melody_charged = { 
    104,  // melody_id
    100, // id to start
    7, // num of notes
   {TF8, TE8, TG8, TA8, T00, TB8, T00},  // tones
   {L32, L32, L32, L32, L64, L16, L00},  // length
   {VHI, VHI, VHI, VHI, VNO, VHI, VNO}   // volume
};

struct melodya beep = { 
    1,  // melody_id
    100, // id to start
    7, // num of notes
   {TF5, T00, T00, T00, T00, T00, T00},  // tones
   {L32, L00, L00, L00, L00, L00, L00},  // length
   {VHI, VNO, VNO, VNO, VNO, VNO, VNO}   // volume
};

struct melodya beep_double = { 
    2,  // melody_id
    100, // id to start
    7, // num of notes
   {TF5, T00, TF5, T00, T00, T00, T00},  // tones
   {L32, L16, L32, L00, L00, L00, L00},  // length
   {VHI, VNO, VHI, VNO, VNO, VNO, VNO}   // volume
};


struct melodya alarm = { 
    3,  // melody_id
    100, // id to start
    7, // num of notes
   {TC4, TC7, TC4, TC7, TC4, TC7, T00},  // tones
   {L04, L04, L04, L04, L04, L04, L00},  // length
   {VHI, VHI, VHI, VHI, VHI, VHI, VNO}   // volume
};

struct melodya horn = { 
    4,  // melody_id
    100, // id to start
    7, // num of notes
   {TF6, T00, TF6, T00, T00, T00, T00},  // tones
   {L16, L16, L02, L00, L00, L00, L00},  // length
   {VHI, VNO, VHI, VNO, VNO, VNO, VNO}   // volume
};

struct melodya short_beep = { 
    5,  // melody_id
    100, // id to start
    7, // num of notes
   {TC8, T00, T00, T00, T00, T00, T00},  // tones
   {L64, L00, L00, L00, L00, L00, L00},  // length
   {VHI, VNO, VNO, VNO, VNO, VNO, VNO}   // volume
};

melodya melody = melody_online;
bool melody_playing = false;
unsigned long millis_play_note = millis();
#define MELODY_BUFFER 4
int melody_buffer[MELODY_BUFFER];




void play_tones();
void play_melody();
int get_melody();
void add_melody();
melodya id_get_melody(int id_m);
void shift_buffer();
