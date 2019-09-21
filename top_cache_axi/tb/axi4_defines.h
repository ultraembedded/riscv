#ifndef AXI4_DEFINES_H
#define AXI4_DEFINES_H

//--------------------------------------------------------------------
// Defines
//--------------------------------------------------------------------
#define AXI4_ADDR_W        32
#define AXI4_DATA_W        32
#define AXI4_AXLEN_W        8
#define AXI4_AXBURST_W      2
#define AXI4_RESP_W         2
#define AXI4_ID_W           4

//--------------------------------------------------------------------
// Enumerations
//--------------------------------------------------------------------
enum eAXI4_BURST
{
    AXI4_BURST_FIXED,
    AXI4_BURST_INCR,
    AXI4_BURST_WRAP
};

enum eAXI4_RESP
{
    AXI4_RESP_OKAY,
    AXI4_RESP_EXOKAY,
    AXI4_RESP_SLVERR,
    AXI4_RESP_DECERR
};

#endif
