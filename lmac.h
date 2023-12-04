
#ifndef NS_LMAC
#define NS_LMAC

//test features described in Journal paper, adaptive listen, etc
//#ifndef JOURNAL_PAPER
//#define JOURNAL_PAPER
//#endif

#include "mac.h"
#include "mac-802_11.h"
#include "cmu-trace.h"
#include "random.h"
#include "timer-handler.h"

/* User-adjustable MAC parameters
 *--------------------------------
 * The default values can be overriden in Each application's Makefile
 * LMAC_MAX_NUM_NEIGHB: maximum number of neighbors.
 * LMAC_MAX_NUM_SCHED: maximum number of different schedules.
 * LMAC_DUTY_CYCLE: duty cycle in percentage. It controls the length of sleep 
 *   interval.
 * LMAC_RETRY_LIMIT: maximum number of RTS retries for sending a single message.
 * LMAC_EXTEND_LIMIT: maximum number of times to extend Tx time when ACK timeout
     happens.
 */

#ifndef LMAC_MAX_NUM_NEIGHBORS
#define LMAC_MAX_NUM_NEIGHBORS 20
#endif

#ifndef LMAC_MAX_NUM_SCHEDULES
#define LMAC_MAX_NUM_SCHEDULES 4
#endif

#ifndef LMAC_DUTY_CYCLE
#define LMAC_DUTY_CYCLE 10
#endif

#ifndef LMAC_RETRY_LIMIT
#define LMAC_RETRY_LIMIT 5
#endif

#ifndef LMAC_EXTEND_LIMIT
#define LMAC_EXTEND_LIMIT 5
#endif

#ifdef JOURNAL_PAPER

#ifndef LMAC_UPDATE_NEIGHB_PERIOD
#define LMAC_UPDATE_NEIGHB_PERIOD 50
#endif
                                                                                                                                                      
#ifndef GUARDTIME
#define GUARDTIME 0.001
#endif

#endif
                                                                                                                                                           

/* Internal MAC parameters
 *--------------------------
 * Do NOT change them unless for tuning S-MAC
 * SYNC_CW: number of slots in the sync contention window, must be 2^n - 1 
 * DATA_CW: number of slots in the data contention window, must be 2^n - 1
 * SYNC_PERIOD: period to send a sync pkt, in cycles.
 * SRCH_CYCLES_LONG: # of SYNC periods during which a node performs a neighbor discovery
 * SRCH_CYCLES_SHORT: if there is no known neighbor, a node need to seach neighbor more aggressively
 */

#define SYNC_CW 31
#define DATA_CW 63
#define SYNCPERIOD 10
#define SYNCPKTTIME 3         // an adhoc value used for now later shld converge with durSyncPkt_

#define SRCH_CYCLES_SHORT 3
#define SRCH_CYCLES_LONG 22


/* Physical layer parameters
 *---------------------------
 * Based on the parameters from PHY_RADIO and RADIO_CONTROL
 * CLOCK_RES: clock resolution in ms. 
 * BANDWIDTH: bandwidth (bit rate) in kbps. Not directly used.
 * PRE_PKT_BYTES: number of extra bytes transmitted before each pkt. It equals
 *   preamble + start symbol + sync bytes.
 * ENCODE_RATIO: output/input ratio of the number of bytes of the encoding
 *  scheme. In Manchester encoding, 1-byte input generates 2-byte output.
 * PROC_DELAY: processing delay of each packet in physical and MAC layer, in ms
 */

#define CLOCKRES 1       // clock resolution is 1ms
#define BANDWIDTH 20      // kbps =>CHANGE BYTE_TX_TIME WHENEVER BANDWIDTH CHANGES
//#define BYTE_TX_TIME 4/10 // 0.4 ms to tx one byte => changes when bandwidth does
#define PRE_PKT_BYTES 5
#define ENCODE_RATIO 2   /* Manchester encoding has 2x overhead */
#define PROC_DELAY 1



// Note everything is in clockticks (CLOCKRES in ms) for tinyOS
// so we need to convert that to sec for ns
#define CLKTICK2SEC(x)  ((x) * (CLOCKRES / 1.0e3))
#define SEC2CLKTICK(x)  ((x) / (CLOCKRES / 1.0e3))


// MAC states
#define SLEEP 0         // radio is turned off, can't Tx or Rx
#define IDLE 1          // radio in Rx mode, and can start Tx
//#define CHOOSE_SCHED 2  // node in boot-up phase, needs to choose a schedule
#define CR_SENSE 2      // medium is free, do it before initiate a Tx
//#define BACKOFF 3       // medium is busy, and cannot Tx
#define WAIT_CTS 3      // sent RTS, waiting for CTS
#define WAIT_DATA 4     // sent CTS, waiting for DATA
#define WAIT_ACK 5      // sent DATA, waiting for ACK
#ifdef JOURNAL_PAPER
#define TX_NEXT_FRAG 6 // send one fragment, waiting for next from upper layer
#else
#define WAIT_NEXTFRAG 6 // send one fragment, waiting for next from upper layer
#endif

#ifdef JOURNAL_PAPER
#define DATA_SENSE1 7 // received a RTS destined to another node, keep listening until confirm sender gets a CTS or starts tx data
#define DATA_SENSE2 8 // received a RTS destined to another node,and did not receive a RTS, keep listening until timeout or receive data
#define TX_PKT 9 // before sending CTS/DATA/ACK, need to wait for a sifs_ time
#endif

// how to send the pkt: broadcast or unicast
#define BCASTSYNC 0
#define BCASTDATA 1
#define UNICAST 2

#ifdef JOURNAL_PAPER
#define UNICAST_ADDR 0
#endif

// Types of pkt
#define DATA_PKT 0
#define RTS_PKT 1
#define CTS_PKT 2
#define ACK_PKT 3
#define SYNC_PKT 4


// radio states for performance measurement
#define RADIO_SLP 0  // radio off
#define RADIO_IDLE 1 // radio idle
#define RADIO_RX 2   // recv'ing mode
#define RADIO_TX 3   // transmitting mode




/*  sizeof lmac datapkt hdr and lmac control and sync packets  */
/*  have been hardcoded here to mirror the values in TINY_OS implementation */
/*  The following is the pkt format definitions for tiny_os implementation */
/*  of lmac : */

/*  typedef struct MAC_CTRLPKT_VALS{ */
/*  unsigned char length; */
/*  char type; */
/*  short addr; */
/*  unsigned char group; */
/*  short srcAddr; */
/*  unsigned char duration; */
/*  short crc; */
/*  }; */

/*  typedef struct MAC_SYNCPKT_VALS{ */
/*  unsigned char length; */
/*  char type; */
/*  short srcAddr; */
/*  short syncNode; */
/*  unsigned char sleepTime;  // my next sleep time from now */
/*  short crc; */
/*  };  */

/*  struct MSG_VALS{ */
/*  unsigned char length; */
/*  char type; */
/*  short addr; */
/*  unsigned char group; */
/*  short srcAddr; */
/*  unsigned char duration; */
/*  char data[DATA_LENGTH]; */
/*  short crc; */
/*  }; */

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

#define SIZEOF_LMAC_DATAPKT 50  // hdr(10) + payload - fixed size pkts
#define SIZEOF_LMAC_CTRLPKT 10
#define SIZEOF_LMAC_SYNCPKT 9  


// Following are the ns definitions of the lmac frames
//SYNC PKT 
struct lmac_sync_frame { 
  int type; 
  int length; 
  int srcAddr;
  //int dstAddr;
  int syncNode; 
  double sleepTime;  // my next sleep time from now */
#ifdef JOURNAL_PAPER
  int state;  // if node has changed schedule
#endif
  int crc; 
}; 

// RTS, CTS, ACK
struct lmac_control_frame {
  int type;
  int length;
  int dstAddr;
  int srcAddr;
  double duration;
  int crc;
};

// DATA 
struct hdr_lmac {
  int type;
  int length;
  int dstAddr;
  int srcAddr;
  double duration;
  //char data[DATA_LENGTH];
  int crc;
};

// Used by lmac when in sync mode
struct LmacSchedTable { 
  int txSync;  // flag indicating need to send sync 
  int txData;  // flag indicating need to send data 
  int numPeriods; // count for number of periods 
#ifdef JOURNAL_PAPER
  int numNodes;  // number of nodes on this schedule
  int syncNode;  // the node who initialized this schedule
  int chkSched; // flag indicating need to check numNodes
#endif
}; 

struct LmacNeighbList { 
  int nodeId; 
  int schedId;
#ifdef JOURNAL_PAPER
  int active; //flag indicating the node is active recently
  int state; // flag indicating the node has changed schedule
#endif 
}; 

class LMAC;

// Timers used in lmac
class LmacTimer : public TimerHandler {
 public:
  LmacTimer(LMAC *a) : TimerHandler() {a_ = a; }
  virtual void expire(Event *e) = 0 ;
  int busy() ;
 protected:
  LMAC *a_;
};

#ifdef JOURNAL_PAPER
// timer for updating neighbors periodically
class LmacUpdateNeighbTimer : public LmacTimer {
 public:
  LmacUpdateNeighbTimer(LMAC *a) : LmacTimer(a) {}
  void expire(Event *e);
};
                                                                                                                                                            
// timer for putting nodes back to sleep after Adaptive Listen
class LmacAdaptiveListenTimer : public LmacTimer {
 public:
  LmacAdaptiveListenTimer(LMAC *a) : LmacTimer(a) {}
  void expire(Event *e);
};
#endif

// Generic timer used for sync, CTS and ACK timeouts
class LmacGeneTimer : public LmacTimer {
 public:
  LmacGeneTimer(LMAC *a) : LmacTimer(a) {}
  void expire(Event *e);
};

// Receive timer for receiving pkts
class LmacRecvTimer : public LmacTimer {
 public:
  LmacRecvTimer(LMAC *a) : LmacTimer(a) { stime_ = rtime_ = 0; }
  void sched(double duration);
  void resched(double time);
  void expire(Event *e);
  double timeToExpire();
 protected:
  double stime_;
  double rtime_;
};

// Send timer
class LmacSendTimer : public LmacTimer {
 public:
  LmacSendTimer(LMAC *a) : LmacTimer(a) {}
  void expire(Event *e);
};

// Nav- indicating if medium is busy or not
class LmacNavTimer : public LmacTimer {
 public:
  LmacNavTimer(LMAC *a) : LmacTimer(a) {}
  void expire(Event *e);
};

// Neighbor nav - if neighbor is busy or not
// used for data timeout
class LmacNeighNavTimer : public LmacTimer {
 public:
  LmacNeighNavTimer(LMAC *a) : LmacTimer(a) { stime_ = rtime_ = 0; }
  void sched(double duration);
  void expire(Event *e);
  double timeToExpire();
 protected:
  double stime_;
  double rtime_;
};

// carrier sense timer
class LmacCsTimer : public LmacTimer {
 public:
  LmacCsTimer(LMAC *a) : LmacTimer(a) {}
  void expire(Event *e);
  void checkToCancel();
};

// synchronisation timer, regulates the sleep/wakeup cycles
class LmacCounterTimer : public LmacTimer { 
 public:  
  friend class LMAC;
  LmacCounterTimer(LMAC *a, int i) : LmacTimer(a) {index_ = i;}
  void sched(double t);
  void expire(Event *e); 
  double timeToSleep();
 protected:
  int index_;
  double value_;
  double syncTime_;
  double dataTime_;
  double listenTime_;
  double sleepTime_;
  double cycleTime_;
  double tts_;
  double stime_;
}; 


// The lmac class
class LMAC : public Mac {
  
  friend class LmacGeneTimer;
  friend class LmacRecvTimer;
  friend class LmacSendTimer;
  friend class LmacNavTimer;
  friend class LmacNeighNavTimer;
  friend class LmacCsTimer; 
  friend class LmacCounterTimer;
#ifdef JOURNAL_PAPER
  friend class LmacUpdateNeighbTimer;
  friend class LmacAdaptiveListenTimer;
#endif

 public:
  LMAC(void);
  ~LMAC() { 
    for (int i=0; i< LMAC_MAX_NUM_SCHEDULES; i++) {
      delete mhCounter_[i];
    }
  }
  void recv(Packet *p, Handler *h);

 protected:
  
  // functions for handling timers
#ifdef JOURNAL_PAPER
  void handleUpdateNeighbTimer();
  void handleAdaptiveListenTimer();
#endif
  void handleGeneTimer();
  void handleRecvTimer();
  void handleSendTimer();
  void handleNavTimer();
  void handleNeighNavTimer();
  void handleCsTimer();
  //void handleChkSendTimer();
  void handleCounterTimer(int i);

  // Internal MAC parameters
  double slotTime_;
  double slotTime_sec_;
  double difs_;
  double sifs_;
  double eifs_;
  double guardTime_;
  double byte_tx_time_;
  double dutyCycle_;
 
 private:
  // functions for node schedule folowing sleep-wakeup cycles
  void setMySched(Packet *syncpkt);
  void sleep();
  void wakeup();

#ifdef JOURNAL_PAPER
  // funtions for update neighbors and schedules
  void check_schedFlag();
  void update_schedTab_neighbList();
  void update_myNeighbList();
  void update_neighbList();
  void checkMySched();
  void dump();
#endif

  // functions for handling incoming packets
  
  void rxMsgDone(Packet* p);
  //void rxFragDone(Packet *p);  no frag for now
#ifdef JOURNAL_PAPER
  void rxFragDone(Packet *p); 
#endif
  void handleRTS(Packet *p);
  void handleCTS(Packet *p);
  void handleDATA(Packet *p);
  void handleACK(Packet *p);
  void handleSYNC(Packet *p);

  // functions for handling outgoing packets
  
  // check for pending data pkt to be tx'ed
  // when lmac is not following SYNC (sleep-wakeup) cycles.
  int checkToSend();               // check if can send, start cs 

  bool chkRadio();         // checks radiostate
  void transmit(Packet *p);         // actually transmits packet

  bool sendMsg(Packet *p, Handler *h);
  bool bcastMsg(Packet *p);
  bool unicastMsg(int n, Packet *p);
  //int sendMoreFrag(Packet *p);
  
  void txMsgDone();
  // void txFragDone();

#ifdef JOURNAL_PAPER
  // functions for handling fragmentation
  bool txNextFrag(void* data);
  void txFragDone();
                                                                                                                                                            
  // functions for handling adaptive listen
  void adaptiveListen();
#endif

  int startBcast();
  int startUcast();
  
  bool sendRTS();
  bool sendCTS(double duration);
  bool sendDATA();
  bool sendACK(double duration);
  bool sendSYNC();

  void sentRTS(Packet *p);
  void sentCTS(Packet *p);
  void sentDATA(Packet *p);
  void sentACK(Packet *p);
  void sentSYNC(Packet *p);
  
  // Misc functions
  void collision(Packet *p);
  void capture(Packet *p);
  double txtime(Packet *p);
  
  void updateNav(double duration);
  void updateNeighNav(double duration);

  void mac_log(Packet *p) {
    logtarget_->recv(p, (Handler*) 0);
  }
  
  void discard(Packet *p, const char* why);
  int drop_RTS(Packet *p, const char* why);
  int drop_CTS(Packet *p, const char* why);
  int drop_DATA(Packet *p, const char* why);
  int drop_SYNC(Packet *p, const char* why);

  // lmac methods to set dst, src and hdr_type in pkt hdrs
  inline int hdr_dst(char* hdr, int dst = -2) {
    struct hdr_lmac *sh = (struct hdr_lmac *) hdr;
    if (dst > -2)
      sh->dstAddr = dst;
    return sh->dstAddr;
  }
  inline int hdr_src(char* hdr, int src = -2) {
    struct hdr_lmac *sh = (struct hdr_lmac *) hdr;
    if (src > -2)
      sh->srcAddr = src;
    return sh->srcAddr;
  }
  inline int hdr_type(char *hdr, u_int16_t type = 0) {
    struct hdr_lmac *sh = (struct hdr_lmac *) hdr;
    if (type)
      sh->type = type;
    return sh->type;
  }
  
  // LMAC internal variables
  
  NsObject*       logtarget_;
  
  // Internal states
  int  state_;                   // MAC state
  int  radioState_;              // state of radio, rx, tx or sleep
  int tx_active_;                
  int mac_collision_;            
  
  int sendAddr_;		// node to send data to
  int recvAddr_;		// node to receive data from
  
  double  nav_;	        // network allocation vector. nav>0 -> medium busy
  double  neighNav_;      // track neighbors' NAV while I'm sending/receiving
  
  // LMAC Timers
#ifdef JOURNAL_PAPER
  LmacUpdateNeighbTimer mhUpdateNeighb_; // timer for updating neighbors periodically
  LmacAdaptiveListenTimer mhAdap_; // timer for putting nodes back to sleep after adaptive listen
#endif
  LmacNavTimer	        mhNav_;		// NAV timer medium is free or not
  LmacNeighNavTimer     mhNeighNav_;    // neighbor NAV timer for data timeout
  LmacSendTimer		mhSend_;	// incoming packets
  LmacRecvTimer         mhRecv_;        // outgoing packets
  LmacGeneTimer         mhGene_;        // generic timer used sync/CTS/ACK timeout
  LmacCsTimer           mhCS_;          // carrier sense timer
  
  // array of countertimer, one for each schedule
  // counter tracking node's sleep/awake cycle
  LmacCounterTimer      *mhCounter_[LMAC_MAX_NUM_SCHEDULES];  


  int numRetry_;	// number of tries for a data pkt
  int numExtend_;      // number of extensions on Tx time when frags are lost
#ifdef JOURNAL_PAPER
  int numFrags_;       // number of fragments in this transmission
  int succFrags_;      // number of successfully transmitted fragments
#endif
  //int numFrags_;       // number of fragments in this transmission
  //int succFrags_;      // number of successfully transmitted fragments
  int lastRxFrag_;     // keep track of last data fragment recvd to prevent duplicate data

  int howToSend_;		// broadcast or unicast
  
  double durSyncPkt_;     // duration of sync packet
  double durDataPkt_;     // duration of data packet XXX caveat fixed packet size
  double durCtrlPkt_;     // duration of control packet
  double timeWaitCtrl_;   // set timer to wait for a control packet
  
  struct LmacSchedTable schedTab_[LMAC_MAX_NUM_SCHEDULES];   // schedule table
  struct LmacNeighbList neighbList_[LMAC_MAX_NUM_NEIGHBORS]; // neighbor list

  int mySyncNode_;                                 // nodeid of my synchronizer
  
  int currSched_;      // current schedule I'm talking to
  int numSched_;       // number of different schedules
  int numNeighb_;      // number of known neighbors
  int numBcast_;       // number of times needed to broadcast a packet
  
  Packet *dataPkt_;		// outgoing data packet
  Packet *pktRx_;               // buffer for incoming pkt
  Packet *pktTx_;               // buffer for outgoing pkt

  // flag to check pending data pkt for tx
  // when lmac is not following SYNC (sleep-wakeup) cycles.
  int txData_ ;

  int syncFlag_;  // is set to 1 when LMAC uses sleep-wakeup cycle
  int selfConfigFlag_;  // is set to 0 when LMAC uses user configurable schedule start time
  double startTime_;  // schedule start time (schedule starts from SYNC period)

  // sleep-wakeup cycle times
  double syncTime_;
  double dataTime_;
  double listenTime_;
  double sleepTime_;
  double cycleTime_;

#ifdef JOURNAL_PAPER
  int adapTime_;  // time before getting back to sleep when doing adaptive listen
  int adaptiveListen_;
  int adapSend_;
  int txRequest_;
  int dataSched_;
  int syncSched_;
  int sendAddr;
                                                                                                                                                            
  int schedState_; // schedule state: first, second schedule...
                                                                                                                                                            
  int globalSchedule_;  // flag indicating if node is in global schedule state
                                                                                                                                                            
  int updateNeighbList_; // flag indicating if node needs to update neighbor list
  char sendSYNCFlag_;    // flag indicating if node has broadcasted SYNC packet or not
#endif

  // neighbor discovery
  int searchNeighb_;  // flag indicating if node is in neighbot discovery period
  int schedListen_;  // flag indicating if node is in scheduled listen period
  int numSync_;  // used to set/clear searchNeighb flag
  
 protected:
  int command(int argc, const char*const* argv);
  virtual int initialized() { 
    return (netif_ && uptarget_ && downtarget_); 
  }
};


#endif //NS_LMAC
