
#include "bitband_ws2812b.h"
#include "WProgram.h"

#define WS2811_TIMING_T0H  60
#define WS2811_TIMING_T1H  176

const uint8_t ones = 0xFF;
volatile uint8_t update_in_progress = 0;
uint32_t update_completed_at = 0;

#define CHUNK_SIZE    1
#define BUFFER_SIZE   (2 * 24 * CHUNK_SIZE)

// Framebuffer - buffer for two chunks of LEDs
uint8_t dmaBitBuffer[BUFFER_SIZE];


uint32_t ledsPerStrip;
uint32_t chunksPerStrip;
volatile uint32_t currentChunk;




void ws2812b_set_pixel(uint8_t *pChunk, uint8_t index, uint8_t strip, uint8_t red, uint8_t green, uint8_t blue);


void gpio_init(void)
{
  // configure the 8 output pins
  GPIOD_PCOR = 0xFF;
  pinMode(2, OUTPUT); // strip #1
  pinMode(14, OUTPUT);  // strip #2
  pinMode(7, OUTPUT); // strip #3
  pinMode(8, OUTPUT); // strip #4
  pinMode(6, OUTPUT); // strip #5
  pinMode(20, OUTPUT);  // strip #6
  pinMode(21, OUTPUT);  // strip #7
  pinMode(5, OUTPUT); // strip #8
}

uint32_t timer_period;

void timer_init(void)
{
  uint32_t frequency = 760000;

  // FROM STM32
  // This computation of pulse length should work ok,
  // at some slower core speeds it needs some tuning.
  // 0,125us period (10 times lower the 1,25us period to have fixed math below)
  //timer_period = F_BUS / frequency;
  //uint32_t cc1 = (10 * timer_period) / 36;
  //uint32_t cc2 = (10 * timer_period) / 15;

  // FROM OCTOWS2811
  timer_period = (F_BUS + frequency / 2) / frequency;
  uint32_t cc1 = (timer_period * WS2811_TIMING_T0H) >> 8;
  uint32_t cc2 = (timer_period * WS2811_TIMING_T1H) >> 8;

  FTM1_SC = 0;
  FTM1_CNT = 0;
  FTM1_MOD = timer_period - 1;
  FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);
  FTM1_C0SC = 0x68; //0x69;
  FTM1_C1SC = 0x68; //0x69;
  FTM1_C0V = cc1;
  FTM1_C1V = cc2;

  // pin 16 triggers DMA(port B) on rising edge
  CORE_PIN16_CONFIG = PORT_PCR_IRQC(1) | PORT_PCR_MUX(3);

  // Start the timer
  FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);
}

void dma_init(void)
{
  // enable clocks to the DMA controller and DMAMUX
  SIM_SCGC7 |= SIM_SCGC7_DMA;
  SIM_SCGC6 |= SIM_SCGC6_DMAMUX;
  DMA_CR = 0;
  DMA_ERQ = 0;

  // DMA channel #1 sets WS2811 high at the beginning of each cycle
  DMA_TCD1_SADDR = &ones;
  DMA_TCD1_SOFF = 0;
  DMA_TCD1_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
  DMA_TCD1_NBYTES_MLNO = 1;
  DMA_TCD1_SLAST = 0;
  DMA_TCD1_DADDR = &GPIOD_PSOR;
  DMA_TCD1_DOFF = 0;
  DMA_TCD1_CITER_ELINKNO = 24 * ledsPerStrip;
  DMA_TCD1_DLASTSGA = 0;
  DMA_TCD1_CSR = DMA_TCD_CSR_DREQ;
  DMA_TCD1_BITER_ELINKNO = 24 * ledsPerStrip;

  // DMA channel #2 writes the pixel data at 20% of the cycle
  DMA_TCD2_SADDR = &dmaBitBuffer;
  DMA_TCD2_SOFF = 1;
  DMA_TCD2_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
  DMA_TCD2_NBYTES_MLNO = 1;
  DMA_TCD2_SLAST = -BUFFER_SIZE;
  DMA_TCD2_DADDR = &GPIOD_PDOR;
  DMA_TCD2_DOFF = 0;
  DMA_TCD2_CITER_ELINKNO = BUFFER_SIZE;
  DMA_TCD2_DLASTSGA = 0;
  DMA_TCD2_CSR = DMA_TCD_CSR_MAJORELINK | DMA_TCD_CSR_MAJORLINKCH(2) | DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_INTHALF;
  DMA_TCD2_BITER_ELINKNO = BUFFER_SIZE;

  // DMA channel #3 clear all the pins low at 48% of the cycle
  DMA_TCD3_SADDR = &ones;
  DMA_TCD3_SOFF = 0;
  DMA_TCD3_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
  DMA_TCD3_NBYTES_MLNO = 1;
  DMA_TCD3_SLAST = 0;
  DMA_TCD3_DADDR = &GPIOD_PCOR;
  DMA_TCD3_DOFF = 0;
  DMA_TCD3_CITER_ELINKNO = 24 * ledsPerStrip;
  DMA_TCD3_DLASTSGA = 0;
  DMA_TCD3_CSR = DMA_TCD_CSR_DREQ | DMA_TCD_CSR_INTMAJOR;
  DMA_TCD3_BITER_ELINKNO = 24 * ledsPerStrip;

  // route the edge detect interrupts to trigger the 3 channels
  DMAMUX0_CHCFG1 = 0;
  DMAMUX0_CHCFG1 = DMAMUX_SOURCE_PORTB | DMAMUX_ENABLE;
  DMAMUX0_CHCFG2 = 0;
  DMAMUX0_CHCFG2 = DMAMUX_SOURCE_FTM1_CH0 | DMAMUX_ENABLE;
  DMAMUX0_CHCFG3 = 0;
  DMAMUX0_CHCFG3 = DMAMUX_SOURCE_FTM1_CH1 | DMAMUX_ENABLE;  

  // enable done interrupts when channel #2 and #3 complete
  NVIC_ENABLE_IRQ(IRQ_DMA_CH2);  
  NVIC_ENABLE_IRQ(IRQ_DMA_CH3);  
}



volatile int counter = 0;
uint8_t outpx[3];

static inline void loadChunkPixels(uint8_t *pChunk, uint32_t chunkLowIndex, uint32_t length) {
  for (uint32_t i = 0, index = chunkLowIndex; i < length; i++, index++) {
    for (uint32_t strip = 0; strip < 8; strip++) {      
      getPixel(strip, index, outpx);
      ws2812b_set_pixel(pChunk, i, strip, outpx[0], outpx[1], outpx[2]);
    }
  }
}

static inline void loadChunkBlack(uint8_t *pChunk, uint32_t chunkLowIndex, uint32_t startIndex, uint32_t endIndex) {  
  memset(pChunk + (startIndex - chunkLowIndex) * 24, 0, (endIndex - startIndex) * 24);
}

static inline void loadFramebufferData(uint32_t chunk)
{
  uint32_t chunkLowIndex = chunk * CHUNK_SIZE;
  uint32_t chunkHighIndex = chunkLowIndex + CHUNK_SIZE;

  uint32_t row = chunk & 1;
  uint8_t *pChunk = &dmaBitBuffer[24 * CHUNK_SIZE * row];  

  if (chunkHighIndex <= ledsPerStrip) {
    // Every index has pixel data
    loadChunkPixels(pChunk, chunkLowIndex, CHUNK_SIZE);
  }
  else if (chunkLowIndex < ledsPerStrip) {
    // Some indices have pixel data
    loadChunkPixels(pChunk, chunkLowIndex, ledsPerStrip - chunkLowIndex);
    loadChunkBlack(pChunk, chunkLowIndex, ledsPerStrip, chunkHighIndex);
  }
  else {
    // All indicies are beyond pixel data
    loadChunkBlack(pChunk, chunkLowIndex, chunkLowIndex, chunkHighIndex);
  }
}

static bool isLoading = false;

static inline bool tryEnterLoadingState() {
  bool didEnterLoadingState;

  noInterrupts();
  if (!isLoading) {
    isLoading = true;
    didEnterLoadingState = true;
  }
  interrupts();

  return didEnterLoadingState;
}

static inline void leaveLoadingState() {
  noInterrupts();
  isLoading = false;
  interrupts();
}

static inline void dma_finish_transfer(void)
{
  DMA_TCD2_CSR &= ~DMA_TCD_CSR_MAJORELINK; 
  DMA_TCD2_CSR |= DMA_TCD_CSR_DREQ;

  digitalWriteFast(LED_BUILTIN, counter % 200 < 100);
  counter++;
}

void dma_ch2_isr(void)
{
  // Clear interrupt
  DMA_CINT = 2;

  currentChunk++;

  if (currentChunk >= chunksPerStrip) {
    dma_finish_transfer();
  }

  if (tryEnterLoadingState()) {
    loadFramebufferData(currentChunk + 1);
    leaveLoadingState();
  }
}

void dma_ch3_isr(void)
{
  // Clear interrupt
  DMA_CINT = 3;

  update_completed_at = micros();
  update_in_progress = 0;
}



void bitband_sendbuf(void)
{
  uint32_t cv = FTM1_C0V;
  noInterrupts();

  // CAUTION: this code is timing critical.
  while (FTM1_CNT <= cv) ;
  while (FTM1_CNT > cv) ; // wait for beginning of an 800 kHz cycle
  while (FTM1_CNT < cv) ;

  FTM1_SC = 0;            // stop FTM1 timer (hopefully before it rolls over)
  FTM1_CNT = 0;

  currentChunk = 0;
  update_in_progress = 1;

  PORTB_ISFR = (1<<0);    // clear any prior rising edge

  uint32_t tmp __attribute__((unused));

  FTM1_C0SC = 0x28;
  tmp = FTM1_C0SC;        // clear any prior timer DMA triggers

  FTM1_C0SC = 0x69;
  FTM1_C1SC = 0x28;

  tmp = FTM1_C1SC;
  FTM1_C1SC = 0x69;

  // Configure DMA
  DMA_TCD2_SADDR = &dmaBitBuffer;
  DMA_TCD2_CSR |= DMA_TCD_CSR_MAJORELINK; 
  DMA_TCD2_CSR &= ~DMA_TCD_CSR_DREQ;

  DMA_ERQ |= 0x0E;     // enable all 3 DMA channels

  FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0); // restart FTM1 timer

  interrupts();
}


#define RAM_BASE 0x20000000
#define RAM_BB_BASE 0x22000000
#define BITBAND_SRAM(address, bit) ( (volatile uint32_t *) (RAM_BB_BASE + (((uint32_t)address) - RAM_BASE) * 32 + (bit) * 4))


void ws2812b_set_pixel(uint8_t *pChunk, uint8_t index, uint8_t strip, uint8_t red, uint8_t green, uint8_t blue)
{
  // Apply gamma
  //red = gammaTable[red];
  //green = gammaTable[green];
  //blue = gammaTable[blue];

  uint32_t invRed = red;
  uint32_t invGreen = green;
  uint32_t invBlue = blue;


  // Bitband optimizations with pure increments, 5us interrupts
  volatile uint32_t *bitBand = BITBAND_SRAM(&pChunk[index * 24], strip);

  // BLUE
  *bitBand = (invBlue >> 7);
  bitBand+=8;

  *bitBand = (invBlue >> 6);
  bitBand+=8;

  *bitBand = (invBlue >> 5);
  bitBand+=8;

  *bitBand = (invBlue >> 4);
  bitBand+=8;

  *bitBand = (invBlue >> 3);
  bitBand+=8;

  *bitBand = (invBlue >> 2);
  bitBand+=8;

  *bitBand = (invBlue >> 1);
  bitBand+=8;

  *bitBand = (invBlue >> 0);
  bitBand+=8;

  // RED
  *bitBand = (invRed >> 7);
  bitBand+=8;

  *bitBand = (invRed >> 6);
  bitBand+=8;

  *bitBand = (invRed >> 5);
  bitBand+=8;

  *bitBand = (invRed >> 4);
  bitBand+=8;

  *bitBand = (invRed >> 3);
  bitBand+=8;

  *bitBand = (invRed >> 2);
  bitBand+=8;

  *bitBand = (invRed >> 1);
  bitBand+=8;

  *bitBand = (invRed >> 0);
  bitBand+=8;

  // GREEN
  *bitBand = (invGreen >> 7);
  bitBand+=8;

  *bitBand = (invGreen >> 6);
  bitBand+=8;

  *bitBand = (invGreen >> 5);
  bitBand+=8;

  *bitBand = (invGreen >> 4);
  bitBand+=8;

  *bitBand = (invGreen >> 3);
  bitBand+=8;

  *bitBand = (invGreen >> 2);
  bitBand+=8;

  *bitBand = (invGreen >> 1);
  bitBand+=8;

  *bitBand = (invGreen >> 0);
  bitBand+=8;
}











void bitband_init(void)
{
  digitalWriteFast(LED_BUILTIN, 0);

  ledsPerStrip = 128;
  chunksPerStrip = (ledsPerStrip + CHUNK_SIZE - 1) / CHUNK_SIZE;

  gpio_init();
  dma_init();
  timer_init();

  for (int i = 0; i < BUFFER_SIZE; i++) {
    dmaBitBuffer[i] = 0;
  }
}

void bitband_show(void)
{
  // wait for any prior show operation
  while (bitband_busy());

  loadFramebufferData(0);
  loadFramebufferData(1);

  bitband_sendbuf();
}

bool bitband_busy(void)
{
    if (update_in_progress) return true;
    // busy for 300 us after dma for WS2811 reset
    if (micros() - update_completed_at < 300) return true;
    return false;
}
