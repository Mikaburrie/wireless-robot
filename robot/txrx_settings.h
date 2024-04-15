// Controls range of transmission rate and margin of error
#define LOW_RATE 80
#define HIGH_RATE 164
#define RX_RATE_MARGIN 20

// Controls range of transmission duty and margin of error
#define LOW_DUTY 100
#define HIGH_DUTY 145
#define RX_DUTY_MARGIN 15

// Defines timeout and sync threshold of receiver
#define RX_SYNC_TIMEOUT 100000
#define RX_SYNC_THRESHOLD 16

#define TX_LOW_RATE LOW_RATE
#define TX_HIGH_RATE HIGH_RATE
#define TX_LOW_DUTY (LOW_DUTY + 3)
#define TX_HIGH_DUTY (HIGH_DUTY + 3)

#define RX_LOW_RATE LOW_RATE
#define RX_HIGH_RATE HIGH_RATE
#define RX_LOW_DUTY LOW_DUTY
#define RX_HIGH_DUTY HIGH_DUTY