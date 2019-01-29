#include "adf4360_regs.hpp"
#include <uhd/types/dict.hpp>
#include <uhd/types/ranges.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/utils/assert_has.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/algorithm.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/usrp/dboard_id.hpp>
#include <uhd/usrp/dboard_base.hpp>
#include <uhd/usrp/dboard_manager.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/math/special_functions/round.hpp> 


// IO Pin functions
#define ADF4360_LOCKDET	   (1 << 0)   //MUXOUT ADF4360 output
#define LT5546_CLIPDET	   (1 << 1)   //CLIPDETECT LT5546 output
#define MyBoard_gpio_pins    ~(ADF4360_LOCKDET ^ LT5546_CLIPDET)	//gpio pins

//Pins direction
#define MyBoard_gpio_out	MyBoard_gpio_pins & (1 << 5) 			//GPIO out pins from motherboard
#define MyBoard_gpio_in	   	MyBoard_gpio_pins & (~ MyBoard_gpio_out)	//RX gpio inputs to mboard

//LED =)
#define RedLED		(1 << 5) & MyBoard_gpio_out		//RED LED =)

using namespace uhd;
using namespace uhd::usrp;
using namespace boost::assign;

/***********************************************************************
 * The MyBoard constants
 **********************************************************************/
static const double MyBoard_freq_center = 100e6;  //68nH,470R;

static const double ref_clock 	= 100e6;		//clock for ADF43601

//static const freq_range_t MyBoard_freq_range((2*MyBoard_freq_center)/2.2, 1.2*(2*MyBoard_freq_center)/2.2);   //Fmax/Fmin = 1.2;

static const std::vector<std::string> MyBoard_antennas = list_of("RF1");

static const uhd::dict<std::string, gain_range_t> MyBoard_gain_ranges = map_list_of
    ("PGA0", gain_range_t(0, 70, 0.022))
;

static const freq_range_t MyBoard_Red_freq_range(double(97e6),double(138e6));  //L1=L2=56nH;
static const freq_range_t MyBoard_Green_freq_range(double(97e6),double(138e6));  //L1=L2=56nH;

//static const freq_range_t MyBoard_Green_freq_range(double(97e6),double(132e6));

/***********************************************************************
 * My dboard class
 **********************************************************************/
class MyBoard: public rx_dboard_base{
public:
    MyBoard(
        ctor_args_t args
    );
    ~MyBoard(void);

private:
    /*!
     * Set the LO frequency for the particular dboard unit.
     * \param unit which unit rx or tx
     * \param target_freq the desired frequency in Hz
     * \return the actual frequency in Hz
     */
    double set_lo_freq(dboard_iface::unit_t unit, double target_freq);

    double set_rx_gain(double gain, const std::string &name);

    /*!
     * Get the lock detect status of the LO.
     * \param unit which unit rx or tx
     * \return sensor for locked
     */
    sensor_value_t get_locked(dboard_iface::unit_t unit){
        const bool locked = (this->get_iface()->read_gpio(unit) & ADF4360_LOCKDET) != 0;
        return sensor_value_t("LO", locked, "locked", "unlocked");
    }

    const uhd::dict<std::string, gain_range_t> rx_gain_ranges;

    const freq_range_t MyBoard_freq_range;

};

/***********************************************************************
 * Register MyBoard (void)
 **********************************************************************/
static dboard_base::sptr make_MyBoard(dboard_base::ctor_args_t args){
    return dboard_base::sptr(new MyBoard(args));
}

UHD_STATIC_BLOCK(reg_MyBoard_dboard){
    dboard_manager::register_dboard(0x0124, &make_MyBoard,  "MyBoard_Red");
    dboard_manager::register_dboard(0x0125, &make_MyBoard,  "MyBoard_Green");
}

/***********************************************************************
 * Structors
 **********************************************************************/
MyBoard::MyBoard(
	ctor_args_t args
): 
	rx_dboard_base(args),
	rx_gain_ranges(MyBoard_gain_ranges),

    MyBoard_freq_range((get_rx_id() == 0x0124)?
        MyBoard_Red_freq_range : MyBoard_Green_freq_range
    )

{
    
    ////////////////////////////////////////////////////////////////////
    // Register properties
    ////////////////////////////////////////////////////////////////////

	if(get_rx_id() == 0x0124) this->get_rx_subtree()->create<std::string>("name").set("MyBoard_Red");
    else if(get_rx_id() == 0x0125) this->get_rx_subtree()->create<std::string>("name").set("MyBoard_Green");
	else this->get_rx_subtree()->create<std::string>("name").set("MyBoard");


    BOOST_FOREACH(const std::string &name, rx_gain_ranges.keys()){
        this->get_rx_subtree()->create<double>("gains/"+name+"/value")
            .set_coercer(boost::bind(&MyBoard::set_rx_gain, this, _1, name))
            .set(rx_gain_ranges[name].start());
        this->get_rx_subtree()->create<meta_range_t>("gains/"+name+"/range")
            .set(rx_gain_ranges[name]);
    }

    this->get_rx_subtree()->create<double>("freq/value")
        .set_coercer(boost::bind(&MyBoard::set_lo_freq, this, dboard_iface::UNIT_RX, _1))
        .set((MyBoard_freq_range.start() + MyBoard_freq_range.stop())/2.0);
    this->get_rx_subtree()->create<meta_range_t>("freq/range")
        .set(MyBoard_freq_range);

    this->get_rx_subtree()->create<std::string>("antenna/value")
        .set(MyBoard_antennas.at(0));
    this->get_rx_subtree()->create<std::vector<std::string> >("antenna/options")
        .set(MyBoard_antennas);

    this->get_rx_subtree()->create<sensor_value_t>("sensors/lo_locked")
        .set_publisher(boost::bind(&MyBoard::get_locked, this, dboard_iface::UNIT_RX));


//this->get_rx_subtree()->create<int>("sensors"); //phony property so this dir exists

    this->get_rx_subtree()->create<std::string>("connection")
        .set("IQ");
    this->get_rx_subtree()->create<bool>("enabled")
        .set(true); //always enabled
    this->get_rx_subtree()->create<bool>("use_lo_offset")
        .set(false);
    this->get_rx_subtree()->create<double>("bandwidth/value") //17MHz low-pass, we want complex double-sided
        .set(2*17e6);
    this->get_rx_subtree()->create<meta_range_t>("bandwidth/range")
        .set(freq_range_t(2*17.0e6, 2*17.0e6));

    //enable RX dboard clock by default
    this->get_iface()->set_clock_enabled(dboard_iface::UNIT_RX, true);


    this->get_iface()->set_clock_rate(dboard_iface::UNIT_RX, ref_clock);


     //set GPIOs
    this->get_iface()->set_pin_ctrl(dboard_iface::UNIT_RX, ~MyBoard_gpio_pins);
    this->get_iface()->set_gpio_ddr(dboard_iface::UNIT_RX, MyBoard_gpio_out);
    this->get_iface()->set_gpio_out(dboard_iface::UNIT_RX, RedLED);

    UHD_MSG(warning) << "Hello, World from MyBoard!I'm registered now!" << std::endl;
}

MyBoard::~MyBoard(void){
}



/***********************************************************************
 * Gain Handling
 **********************************************************************/
static double rx_pga0_gain_to_dac_volts(double &gain, double range){

    static const double max_volts = 1.5, min_volts = 0;
    static const double slope = (max_volts-min_volts)/(range);

    //calculate the voltage for the aux dac
    double dac_volts = uhd::clip<double>(gain*slope + min_volts, min_volts, max_volts);

    //the actual gain setting
    gain = (dac_volts - min_volts)/slope;

    return dac_volts;
}

double MyBoard::set_rx_gain(double gain, const std::string &name){
    assert_has(rx_gain_ranges.keys(), name, "MyBoard gain name");
    if(name == "PGA0"){
        double dac_volts = rx_pga0_gain_to_dac_volts(gain, 
                              (rx_gain_ranges["PGA0"].stop() - rx_gain_ranges["PGA0"].start()));

        //write the new voltage to the aux dac
        this->get_iface()->write_aux_dac(dboard_iface::UNIT_RX, dboard_iface::AUX_DAC_A, dac_volts);

        return gain;
    }
    else UHD_THROW_INVALID_CODE_PATH();
}


/***********************************************************************
 * Tuning
 **********************************************************************/
void sleep( int msec ) { 
	boost::xtime xt; 
	boost::xtime_get(&xt, boost::TIME_UTC_); 
	xt.sec += msec/1000; 
	xt.nsec += msec * 1000000; 
	boost::thread::sleep(xt); 
}

double MyBoard::set_lo_freq(
    dboard_iface::unit_t unit,
    double target_freq
){
    UHD_LOGV(often) << boost::format(
        "MyBoard tune: target frequency %f Mhz"
    ) % (target_freq/1e6) << std::endl;

    //clip the input
    target_freq = MyBoard_freq_range.clip(target_freq);


     /*
      fvco = B*fref/R
     */

	double actual_freq=0;
	int R=100, B = 100;

	B = boost::math::iround(2*R*target_freq/ref_clock);

	actual_freq = double(B)*ref_clock/(2*double(R));

    UHD_MSG(warning) << boost::format(
        "MyBoard tune: R=%d, B=%d"
    ) % R % B  << std::endl;



   //load the register values
    adf4360_regs_t regs;
    regs.core_power_level        = adf4360_regs_t::CORE_POWER_LEVEL_5MA;
    regs.counter_operation       = adf4360_regs_t::COUNTER_OPERATION_NORMAL;
    regs.muxout_control          = adf4360_regs_t::MUXOUT_CONTROL_DLD;
    regs.phase_detector_polarity = adf4360_regs_t::PHASE_DETECTOR_POLARITY_POS;
    regs.charge_pump_output      = adf4360_regs_t::CHARGE_PUMP_OUTPUT_NORMAL;
    regs.cp_gain_0               = adf4360_regs_t::CP_GAIN_0_SET1;
    regs.mute_till_ld            = adf4360_regs_t::MUTE_TILL_LD_DIS;
    regs.output_power_level      = adf4360_regs_t::OUTPUT_POWER_LEVEL_7_5MA;
    regs.current_setting1        = adf4360_regs_t::CURRENT_SETTING1_0_31MA;
    regs.current_setting2        = adf4360_regs_t::CURRENT_SETTING2_0_31MA;
    regs.power_down              = adf4360_regs_t::POWER_DOWN_NORMAL_OP;
    regs.prescaler_value         = adf4360_regs_t::PRESCALER_VALUE_8_9;
    regs.a_counter               = 0;
    regs.b_counter               = B;
    regs.cp_gain_1               = adf4360_regs_t::CP_GAIN_1_SET1;
    regs.divide_by_2_output      = adf4360_regs_t::DIVIDE_BY_2_OUTPUT_FUND;
    regs.divide_by_2_prescaler   = adf4360_regs_t::DIVIDE_BY_2_PRESCALER_FUND;
    regs.r_counter               = R;
    regs.ablpw                   = adf4360_regs_t::ABLPW_3_0NS;
    regs.lock_detect_precision   = adf4360_regs_t::LOCK_DETECT_PRECISION_3CYCLES;
    regs.test_mode_bit           = 0;
    regs.band_select_clock_div   = adf4360_regs_t::BAND_SELECT_CLOCK_DIV_1;


    //write the registers
    std::vector<adf4360_regs_t::addr_t> addrs = list_of //correct power-up sequence to write registers (R, C, N)
        (adf4360_regs_t::ADDR_RCOUNTER)
        (adf4360_regs_t::ADDR_CONTROL)
        (adf4360_regs_t::ADDR_NCOUNTER);
	

    
    BOOST_FOREACH(adf4360_regs_t::addr_t addr, addrs){
        this->get_iface()->write_spi(
            unit, spi_config_t::EDGE_RISE,
            regs.get_reg(addr), 24
        );
//	sleep(15);
    }

    //return the actual frequency
    UHD_LOGV(often) << boost::format(
        "MyBoard tune: actual frequency %f Mhz"
    ) % (actual_freq/1e6) << std::endl;
    return actual_freq;
}

