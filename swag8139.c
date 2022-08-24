/*
 *
 *	Realtek RTL 8139 networking driver.
 *
 *	Written by: Gavrilo Palalic
 *		
 *	(C) Copyright 2022, Gavrilo Palalic
 *		
 *	Contact: gavrilopalalic@protonmail.com
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/stddef.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <linux/etherdevice.h>
#define TOTAL_TX_BUF_SIZE 1536*4
//SOME CONSTANTS DEFINED FOR RTL8139
//
////some frequently used register addrs
#define CPCR 0xE0 //c+ mode command register
#define CMDR 0x37 //normal command register
#define DTCCR 0x10 //tally dump registers, 8 bytes long
#define IMR 0x3C //interrupt mask registers, 2 bytes long
#define ISR 0x3E //interrupt status registers, 2 bytes long
#define RCR 0x44 //Rx config registers, 4 bytes long
#define RDSAR 0xE4 //write where the rx descriptors are allocated here, 64 bit le addr
#define NUM_TX_DESC 4
//various bit values
#define DAC_BIT (1<<4) //DAC bit in C+ command register
#define DTCCR_CMD_BIT (1<<3)	//set to 1 to start tally counter dump,
				//then read until set back to 0 by device
#define CMD_RES_BIT (1<<4) 	//set to 1 to reset the device
				//(needed during init to start with a clean slate)
#define CMD_TX_EN_BIT (1<<2) //enable TX in command register
#define CMD_RX_EN_BIT (1<<3) //enable RX in command register
#define CP_TX_EN_BIT 1 //enable TX in C+ mode
#define CP_RX_EN_BIT (1<<1) //enable RX in C+ mode
#define ISR_ROK 1 //raised if "new packet received" interrupt is issued (RX OK)
#define IMR_ROK 1 //mask "new packet received" interrupt
#define RCR_AB (1<<3) //accept broadcast packets
#define RCR_APM (1<<1) //accept packets with matching destination MAC address
#define RCR_AALL 0x3F //accept all packets
#define RXDF1_OWN (1<<31) //own bit in receive descriptor
#define RXDF1_EOR (1<<30) //end of ring bit in receive descriptor
#define RXDF1_BUFSIZE 0x0 //first 13 bits

//some sizes
#define TALLY_DUMP_SIZE 64 	//size of tally dump
//#define RX_BUF_LEN ( 8192<<2)				//(its contents are in ch6.3 (DTCCR) in datasheet)

#define RX_BUF_LEN_IDX 2         /* 0==8K, 1==16K, 2==32K, 3==64K */
#define RX_BUF_LEN     (8192 << RX_BUF_LEN_IDX)
#define RX_BUF_PAD     16           /* see 11th and 12th bit of RCR: 0x44 */
#define RX_BUF_WRAP_PAD 2048   /* spare padding to handle pkt wrap */
#define RX_BUF_TOT_LEN  (RX_BUF_LEN + RX_BUF_PAD + RX_BUF_WRAP_PAD)
#define TX_BUF_SIZE 1536


#define NUM_RX_BUFS	64	//number of rx buffers for packets
#define RX_DESC_SIZE	16 	//2*32 bit flags and an 64 bit rx buffer address
#define RX_BUF_SIZE	1536	//size of each rx buffer

//
//END OF CONSTANTS
#define VENDOR_ID  0x10EC
#define DEVICE_ID   0x8139
#define DRIVER_VER "0.01A"
#define TxStatOK      0x8000
#define TxStatOk      0x8000
#define RxBufEmpty 0x01
#define CmdTxEnb   0x04
#define CmdRxEnb   0x08
#define CmdReset   0x10
#define RxOK       0x01
#define RxErr      0x02
#define TxOK       0x04
#define TxErr      0x08
#define RxOverFlow 0x10
#define RxUnderrun 0x20
#define RxFIFOOver 0x40
#define CableLen   0x2000
#define TimeOut    0x4000
#define SysErr     0x8000
#define MINIMUM_PACKET_SIZE 
#define INT_MASK (RxOK | RxErr | TxOK | TxErr | \
               RxOverFlow | RxUnderrun | RxFIFOOver | \
               CableLen | TimeOut | SysErr)
#define TSD0          0x10
#define TSAD0       0x20
#define RBSTART  0x30
#define TxUnderrun    0x4000
#define TxStatOK      0x8000
#define TxOutOfWindow 0x20000000
#define TxAborted     0x40000000



MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gavrilo Palalic");
MODULE_DESCRIPTION("Very basic RTL8139 driver.");
MODULE_VERSION(DRIVER_VER);

typedef struct swag8139{
	struct pci_dev *dev;
	void *mmio;
	unsigned long regs_len; /* length of I/O or MMI/O region */
        unsigned int tx_flag;
        unsigned int cur_tx;
        unsigned int dirty_tx;
        unsigned char *tx_buf[NUM_TX_DESC];   /* Tx bounce buffers */
        unsigned char *tx_bufs;        /* Tx bounce buffer region. */
        dma_addr_t tx_bufs_dma;
	

 	dma_addr_t rx_dma;
	unsigned char  *rx_ring;
	unsigned int cur_rx;
	struct net_device_stats stats;

}swag8139_dev;


//static struct pci_dev *dev;
static struct net_device *net;


static struct pci_device_id  ioids[]={
	{PCI_DEVICE(VENDOR_ID,DEVICE_ID)},
	{ }
};

MODULE_DEVICE_TABLE(pci,ioids);
swag8139_dev *gandalf;
static u64 mac_read(void __iomem *regs){
	uint32_t mac1,mac2,mac_whole;
	mac1 = ioread32(regs);
	mac2 = ioread32(regs + 4);

	mac_whole = ((u64) mac2 << 32) | mac1;
	printk("swag8139: [T] MAC :%p",&mac_whole);
	return mac_whole;


}
static inline void eth_copy_and_sum (struct sk_buff *dest, 
				     const unsigned char *src, 
				     int len, int base)
{
	memcpy (dest->data, src, len);
}
static irqreturn_t rtl_handler(int irq, void *dev_instance){

	struct net_device *dev = (struct net_device*)dev_instance;
    	swag8139_dev *tp = netdev_priv(dev);
        void *ioaddr = tp->mmio;
        unsigned short isr = readw(ioaddr + ISR);
        
        /* clear all interrupt.
         * Specs says reading ISR clears all interrupts and writing
         * has no effect. But this does not seem to be case. I keep on
         * getting interrupt unless I forcibly clears all interrupt :-(
         */

        writew(0xffff, ioaddr + ISR);
		printk("\nreceived interrupt.");
        if((isr & TxOK) || (isr & TxErr)) 
        {
               while((tp->dirty_tx != tp->cur_tx) || netif_queue_stopped(dev))
               {
                       unsigned int txstatus = 
                               readl(ioaddr + TSD0 + tp->dirty_tx * sizeof(int));

                       if(!(txstatus & (TxStatOK | TxAborted | TxUnderrun)))
                               break; /* yet not transmitted */

                       if(txstatus & TxStatOK) {
				printk("\n good send happened.");
                        	tp->stats.tx_bytes += (txstatus & 0x1fff);
                       		tp->stats.tx_packets++;
				tp->stats.collisions+= (txstatus >>24 ) & 15;
		       }
                       else {
                              
                               tp->stats.tx_errors++;
                       }
                        
                       tp->dirty_tx++;
                       tp->dirty_tx = tp->dirty_tx % NUM_TX_DESC;

                       if((tp->dirty_tx == tp->cur_tx) & netif_queue_stopped(dev))
                       {
                        
							   
                               netif_wake_queue(dev);
                       }
               }
        }

        if(isr & RxErr) {
               /* TODO: Need detailed analysis of error status */
               
               tp->stats.rx_errors++;
        }

        if(isr & RxOK) {
               printk("\nreceived RX");
               while((readb(ioaddr + CMDR) & RxBufEmpty) == 0)
               {
                       unsigned int rx_status;
                       unsigned short rx_size;
                       unsigned short pkt_size;
                       struct sk_buff *skb;

                       if(tp->cur_rx > RX_BUF_LEN)
                               tp->cur_rx = tp->cur_rx % RX_BUF_LEN;
        
                       /* TODO: need to convert rx_status from little to host endian
                        * XXX: My CPU is little endian only :-)
                        */
                       rx_status = *(unsigned int*)(tp->rx_ring + tp->cur_rx);
                       rx_size = rx_status >> 16;
                       
                       /* first two bytes are receive status register 
                        * and next two bytes are frame length
                        */
                       pkt_size = rx_size - 4;

                       /* hand over packet to system */
                       skb = dev_alloc_skb (pkt_size + 2);
                       if (skb) {
							
                               skb->dev = dev;
                               skb_reserve (skb, 2); /* 16 byte align the IP fields */

                               skb_copy_to_linear_data(skb,&tp->rx_ring+tp->cur_rx+4,pkt_size);
                               skb_put (skb, pkt_size);
                               skb->protocol = eth_type_trans (skb, dev);
                               //netif_rx (skb);
								netif_receive_skb(skb);
//                               dev->last_rx = jiffies;

                               tp->stats.rx_bytes += pkt_size;
                               tp->stats.rx_packets++;
                       } 
                       else {
                              
                               tp->stats.rx_dropped++;
                       }
               
                      
			 			 tp->cur_rx = (tp->cur_rx + rx_size + 4 + 3) & ~3;

                       writew(tp->cur_rx, ioaddr + 0x38);
               }
        }
		writew(0x5,ioaddr+0x3e);

	return IRQ_HANDLED;
}
//tx ring initialize (initing buffer where our packet will be , before transformed)
static void init_ring(struct net_device* dev){

		swag8139_dev *tmp = netdev_priv(dev);
		volatile int i;
		tmp->cur_tx = 0;
		tmp->dirty_tx = 0;
		tmp->cur_rx=0;

		for(i =0;i<4;i++){
			tmp->tx_buf[i]=&tmp->tx_bufs[i*1536];
		}
		printk("\nHL:%x",tmp->dev->vendor);

}
#define CFG93 0x50
enum Cfg9346Bits {
	Cfg9346_Lock	= 0x00,
	Cfg9346_Unlock	= 0xC0,
};
static void rtl_start_hw(struct net_device* dev){

	swag8139_dev *temp = netdev_priv(dev);
	volatile int i =0 ;
	void __iomem  *addr = temp->mmio;
	//wake up device from low power mode
	outb(0x00,addr+0x52);
	//reset device
	outb(0x10,addr+0x37);
	while(!(inb(addr+0x37)& 0x10)){
	}
	//end of reset
	//enable TX pin (pull TX high)
	writeb(Cfg9346_Unlock,addr+0x50);
	writeb(CmdTxEnb | CmdRxEnb ,addr+CMDR);
	writel(((1 << 12) | (7 << 8) | (1 << 7) |
                               (1 << 3) | (1 << 2) | (1 << 1)), addr + 0x44);

	writel(0x00000600,addr+0x40);
	for(i =0;i<4;i++){
		writel(temp->tx_bufs_dma+(temp->tx_buf[i]-temp->tx_bufs),addr+TSAD0+(i*4));
	}
	writel(temp->rx_dma,addr+RBSTART);
	//missed packet counter ( YOU CANNOT MISS, SO SET TO 0 ! ) 
	writel(0, addr + 0x4c);
	//no early interrupts for rx, thanks
	writew((readw(addr+0x5c) & 0xF000),addr+0x5c);

	unsigned int tempo = ioread8(addr+0x37);
	if ((!(tempo & CmdRxEnb)) || (!(tempo & CmdTxEnb)))
	{
		writeb(CmdTxEnb | CmdRxEnb ,addr+CMDR);
	}
	writew(INT_MASK,addr+IMR);
}

static int rtl_open(struct net_device * dev){
	printk("rtl_open() called");
	
	swag8139_dev *temp = netdev_priv(dev);
	if(request_irq(dev->irq,rtl_handler,0,dev->name,dev)){
		printk("swag8139: could not request irq.");
		return -1;
	}
	printk("\nallocated dumbass");
	temp->tx_bufs =pci_alloc_consistent(temp->dev,TOTAL_TX_BUF_SIZE,&temp->tx_bufs_dma);
	if(!temp->tx_bufs){
		printk("swag8139: could not create DMA mem.");
		free_irq(dev->irq,dev);
		return -12;
	}
	temp->rx_ring=pci_alloc_consistent(temp->dev,RX_BUF_TOT_LEN,&temp->rx_dma);
	if(!temp->rx_ring){
		printk("swag8139: could not allocate DMA for RX");
		return -12;
	}
	//TODO
	/*
	if((!tp->tx_bufs)  || (!tp->rx_ring)) {
        free_irq(dev->irq, dev);

        if(tp->tx_bufs) {
                       pci_free_consistent(tp->pci_dev, TOTAL_TX_BUF_SIZE, tp->tx_bufs, tp->tx_bufs_dma);
                       tp->tx_bufs = NULL;
               }
        if(tp->rx_ring) {
                       pci_free_consistent(tp->pci_dev, RX_BUF_TOT_LEN, tp->rx_ring, tp->rx_ring_dma);
                       tp->rx_ring = NULL;
               }
        return -ENOMEM;
}
      
	*/
	temp->tx_flag=0;		
	init_ring(dev);
	rtl_start_hw(dev);
	netif_start_queue(dev);

	return 0;

}
static int rtl_stop(struct net_device * dev){
	printk("rtl_stop() called");
	free_irq(dev->irq,dev);
	
	
	return 0;

}
static int rtl_xmit(struct sk_buff *skb,struct net_device * dev){
	printk("rtl_xmit() clld");
	

	swag8139_dev *tm = netdev_priv(dev);
	void __iomem *addr = tm->mmio;
	unsigned int entry = tm->cur_tx;

	unsigned int len = skb->len;
	if(len<TX_BUF_SIZE){
		if(len<60){
			memset(tm->tx_buf[entry],0,60);

		}
		skb_copy_and_csum_dev(skb,tm->tx_buf[entry]);
		dev_kfree_skb(skb);

	}
	else{
		dev_kfree_skb(skb);
		return 0;
	}
	wmb();
	writel(tm->tx_flag | max(len,(unsigned int)60),addr+TSD0+(entry*sizeof(uint32_t)));
	entry++;

	tm->cur_tx =entry%4;
	if((tm->cur_tx-4) == tm->dirty_tx){
		netif_stop_queue(dev);
	}
	return 0;

}
static struct net_device_stats * rtl_stats(struct net_device *dev){

	swag8139_dev *tmp = netdev_priv(dev);
	return &(tmp->stats);

}
static struct net_device_ops swag_ndo = {
	.ndo_open = rtl_open,
	.ndo_stop = rtl_stop,
	.ndo_start_xmit = rtl_xmit,
	.ndo_get_stats = rtl_stats,

};

static int rtlprobe(struct pci_dev *device ,const  struct pci_device_id *id){
	printk("\nswag8139: loaded vendor_id:%x",device->vendor);
	swag8139_dev *swag;


	unsigned long mmio_start, mmio_end, mmio_len, mmio_flags;
	struct net_device *loader;
	loader = alloc_etherdev(sizeof(*swag));
	if(!loader){
		printk("swag8139: [E] could not allocate ethernet device.");
		return -12;
	}

	//TODO , create struct device*
	//SET_NETDEV_DEV(net,	device);

	swag = netdev_priv(loader);
	swag->dev = device;

	net = loader;
	memcpy(net,loader,sizeof(*swag));


	//init other shit

	pci_set_drvdata(device,swag);

	if(pci_enable_device(device)){
		printk("swag8139: could not enable rtl8139.");
		return -12;
	}

	mmio_start = pci_resource_start(device,1);
	mmio_end = pci_resource_end(device,1);
	mmio_len = pci_resource_len(device,1);
	mmio_flags = pci_resource_flags(device,1);
	if(pci_request_regions(device,"swag8139")){
		printk("swag8139: could not read pci regions.");
		return -12;
	}
	pci_set_master(device);
	if(!mmio_start){
		printk("swag8139: could not get resources.");
		return -12;
	}
	swag->mmio =ioremap(mmio_start,mmio_len);
	//taken from 8139too.ko
	//swag->mmio = pci_iomap(device,0,mmio_len);

	net->base_addr = (long unsigned int)swag->mmio;
	
	if(!swag->mmio || !net->base_addr){
		printk("swag8139: could not remap.");
		return -12;
	}
	
	
	//wakeup from low power mode  and reset device 
	outb(0x00,swag->mmio+0x52);
	outb(0x10,swag->mmio+0x37);
	while(!(inb(swag->mmio+0x37)& 0x10)){
	}
	//device has been reset if you proceed onto next shit

	printk("swag8139: stage 2: registering netdev.");

	//read mac address 
	volatile int i =0;
	for(i = 0;i<6;i++){
		net->dev_addr[i] = readb(net->base_addr+i);
		net->broadcast[i] = 0xff;
	}
	
	//testing some shit from other drivers.
	//
	net->hard_header_len=14;
	//char *drv = "jocadoca";
	//memcpy(net->name,drv,8);
	

	net->irq = device->irq;
	net->netdev_ops = &swag_ndo;
	//net->state =	__LINK_STATE_PRESENT;


	//registering device into our network stack
	//its so much better to use this struct net_device than to have to recompile the whole fucking kernel with changes to use my driver.
	//SMART LINUS!
	if(register_netdev(net)){
		printk("swag8139: [NET]  could not register network device :(");
		return -12;

	}

	return 0; 
}

static void rtlremove(struct pci_dev *dev ){
	printk("removing swag8139 driver...");	
	iounmap(net->base_addr);
	//free_irq(net->irq,net);
	pci_release_regions(dev);
	unregister_netdev(net);

	pci_disable_device(dev);
		
}
static struct pci_driver rtldriver={

	.name="swag8139",
	.id_table = ioids,
	.probe = rtlprobe,
	.remove = rtlremove,
};

static int __init rtlinit(void){
	return pci_register_driver(&rtldriver);

}

static void __exit rtlexit(void){
	printk("swag8139: driver unloaded");
	pci_unregister_driver(&rtldriver);
}
module_init(rtlinit);
module_exit(rtlexit);

