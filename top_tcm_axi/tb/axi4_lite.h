#ifndef AXI4_LITE_H
#define AXI4_LITE_H

#include <systemc.h>

//----------------------------------------------------------------
// Interface (master)
//----------------------------------------------------------------
class axi4_lite_master
{
public:
    // Members
    sc_uint <1> AWVALID;
    sc_uint <32> AWADDR;
    sc_uint <1> WVALID;
    sc_uint <32> WDATA;
    sc_uint <4> WSTRB;
    sc_uint <1> BREADY;
    sc_uint <1> ARVALID;
    sc_uint <32> ARADDR;
    sc_uint <1> RREADY;

    // Construction
    axi4_lite_master() { init(); }

    void init(void)
    {
        AWVALID = 0;
        AWADDR = 0;
        WVALID = 0;
        WDATA = 0;
        WSTRB = 0;
        BREADY = 0;
        ARVALID = 0;
        ARADDR = 0;
        RREADY = 0;
    }

    bool operator == (const axi4_lite_master & v) const
    {
        bool eq = true;
        eq &= (AWVALID == v.AWVALID);
        eq &= (AWADDR == v.AWADDR);
        eq &= (WVALID == v.WVALID);
        eq &= (WDATA == v.WDATA);
        eq &= (WSTRB == v.WSTRB);
        eq &= (BREADY == v.BREADY);
        eq &= (ARVALID == v.ARVALID);
        eq &= (ARADDR == v.ARADDR);
        eq &= (RREADY == v.RREADY);
        return eq;
    }

    friend void sc_trace(sc_trace_file *tf, const axi4_lite_master & v, const std::string & path)
    {
        sc_trace(tf,v.AWVALID, path + "/awvalid");
        sc_trace(tf,v.AWADDR, path + "/awaddr");
        sc_trace(tf,v.WVALID, path + "/wvalid");
        sc_trace(tf,v.WDATA, path + "/wdata");
        sc_trace(tf,v.WSTRB, path + "/wstrb");
        sc_trace(tf,v.BREADY, path + "/bready");
        sc_trace(tf,v.ARVALID, path + "/arvalid");
        sc_trace(tf,v.ARADDR, path + "/araddr");
        sc_trace(tf,v.RREADY, path + "/rready");
    }

    friend ostream& operator << (ostream& os, axi4_lite_master const & v)
    {
        os << hex << "AWVALID: " << v.AWVALID << " ";
        os << hex << "AWADDR: " << v.AWADDR << " ";
        os << hex << "WVALID: " << v.WVALID << " ";
        os << hex << "WDATA: " << v.WDATA << " ";
        os << hex << "WSTRB: " << v.WSTRB << " ";
        os << hex << "BREADY: " << v.BREADY << " ";
        os << hex << "ARVALID: " << v.ARVALID << " ";
        os << hex << "ARADDR: " << v.ARADDR << " ";
        os << hex << "RREADY: " << v.RREADY << " ";
        return os;
    }

    friend istream& operator >> ( istream& is, axi4_lite_master & val)
    {
        // Not implemented
        return is;
    }
};

#define MEMBER_COPY_AXI4_LITE_MASTER(s,d) do { \
    s.AWVALID = d.AWVALID; \
    s.AWADDR = d.AWADDR; \
    s.WVALID = d.WVALID; \
    s.WDATA = d.WDATA; \
    s.WSTRB = d.WSTRB; \
    s.BREADY = d.BREADY; \
    s.ARVALID = d.ARVALID; \
    s.ARADDR = d.ARADDR; \
    s.RREADY = d.RREADY; \
    } while (0)

//----------------------------------------------------------------
// Interface (slave)
//----------------------------------------------------------------
class axi4_lite_slave
{
public:
    // Members
    sc_uint <1> AWREADY;
    sc_uint <1> WREADY;
    sc_uint <1> BVALID;
    sc_uint <2> BRESP;
    sc_uint <1> ARREADY;
    sc_uint <1> RVALID;
    sc_uint <32> RDATA;
    sc_uint <2> RRESP;

    // Construction
    axi4_lite_slave() { init(); }

    void init(void)
    {
        AWREADY = 0;
        WREADY = 0;
        BVALID = 0;
        BRESP = 0;
        ARREADY = 0;
        RVALID = 0;
        RDATA = 0;
        RRESP = 0;
    }

    bool operator == (const axi4_lite_slave & v) const
    {
        bool eq = true;
        eq &= (AWREADY == v.AWREADY);
        eq &= (WREADY == v.WREADY);
        eq &= (BVALID == v.BVALID);
        eq &= (BRESP == v.BRESP);
        eq &= (ARREADY == v.ARREADY);
        eq &= (RVALID == v.RVALID);
        eq &= (RDATA == v.RDATA);
        eq &= (RRESP == v.RRESP);
        return eq;
    }

    friend void sc_trace(sc_trace_file *tf, const axi4_lite_slave & v, const std::string & path)
    {
        sc_trace(tf,v.AWREADY, path + "/awready");
        sc_trace(tf,v.WREADY, path + "/wready");
        sc_trace(tf,v.BVALID, path + "/bvalid");
        sc_trace(tf,v.BRESP, path + "/bresp");
        sc_trace(tf,v.ARREADY, path + "/arready");
        sc_trace(tf,v.RVALID, path + "/rvalid");
        sc_trace(tf,v.RDATA, path + "/rdata");
        sc_trace(tf,v.RRESP, path + "/rresp");
    }

    friend ostream& operator << (ostream& os, axi4_lite_slave const & v)
    {
        os << hex << "AWREADY: " << v.AWREADY << " ";
        os << hex << "WREADY: " << v.WREADY << " ";
        os << hex << "BVALID: " << v.BVALID << " ";
        os << hex << "BRESP: " << v.BRESP << " ";
        os << hex << "ARREADY: " << v.ARREADY << " ";
        os << hex << "RVALID: " << v.RVALID << " ";
        os << hex << "RDATA: " << v.RDATA << " ";
        os << hex << "RRESP: " << v.RRESP << " ";
        return os;
    }

    friend istream& operator >> ( istream& is, axi4_lite_slave & val)
    {
        // Not implemented
        return is;
    }
};

#define MEMBER_COPY_AXI4_LITE_SLAVE(s,d) do { \
    s.AWREADY = d.AWREADY; \
    s.WREADY = d.WREADY; \
    s.BVALID = d.BVALID; \
    s.BRESP = d.BRESP; \
    s.ARREADY = d.ARREADY; \
    s.RVALID = d.RVALID; \
    s.RDATA = d.RDATA; \
    s.RRESP = d.RRESP; \
    } while (0)


#endif
