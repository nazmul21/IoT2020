/******************************************************************************/

/* File - app.c
*  
*  Target Hardware: SIEMENS IoT2020
*  
*  This module used to acquire data from different sensors (DS18B20, HSG-20G, 
*  GP2Y1010AU) using Intel low level skeleton mraa library. Acquired data then
*  store to SQLite database for generation of trend as well as other function
*  tailored to application of safety assistant for industrial control system.
*   
*  Version - 1.0 (May, 2017)
*/

/******************************************************************************/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <assert.h>
#include <sqlite3.h>

#include "mraa.h"

/******************************************************************************/

/* The predefine symbol below 'RUN_TIME_LOG' used to print runtime data such as
*  DS18B20 ROM content, temperature value, sqlite data stored successfully 
*  etc. If you need to print those, then uncomment the below and build again.  
*/
//#define RUN_TIME_LOG

#define SUCCESS    (1)
#define FAIL       (0)

/* Define DS18B20 search type in order to issue as new search (new iteration) 
*  or continue with previous search (required for multiple sensors)
*/
#define NEW_SEARCH                  (1)
#define CONTINUE_WITH_PREV_SEARCH   (0)

/* Command to to issue read scratchpad and start temperature conversion 
*  respectively from DS18B20
*/
#define CMD_READ_SCRATCHPAD         (0xBE)
#define CMD_START_TEMP_CONV         (0x44)

/* Define the size of address ROM (8 bytes) of DS18B20 family. Define the number
*  of sensor(s) used in application. For me - 2 sensors used
*/
#define DS18B20_ADDR_LEN            (MRAA_UART_OW_ROMCODE_SIZE)
#define NUM_OF_SENSORS              (2)

/* SQLite3 database path to store temperature for future usage
*/
#define DATABASE_PATH               "/home/root/ctrl_room_monitor/database/ctrl_db.db"

/******************************************************************************/

/******************************************************************************/

/* Definition of enumerated type with the keywords, will be used in this application
*/
typedef enum 
{
	NONE = 0, 
	START_CONV,
	WAIT_TILL_CONV_FINISHED,
	READ_TEMP,
	READ_DUST_CONCENTRATION,
	READ_HUMIDITY,
	WAIT
} app_states_t;

/******************************************************************************/

/* Function declaration to read temperature data from DS18B20. In this application 
*  the default resolution (12-bit) is used.
*  @param[in] uart_path - UART instance returned by  mraa_uart_ow_init() function
*  @param[in] sen_addr  - 8-byte ROM address of DS18B20
*  @param[in] p_temp    - a float pointer which contains temperature in degree celsius
*                         upon a valid read 
*  @return - uint8_t ( SUCCESS(1), FAIL(0) )
*/
static uint8_t ds18b20_read_temp_float(mraa_uart_ow_context uart_path, uint8_t 
                                        sen_addr[DS18B20_ADDR_LEN], float * p_temp);

/* Function declaration to update i.e. start conversion in this case a specified DS18B20 
*  with its address. 
*  @param[in] uart_path - UART instance returned by  mraa_uart_ow_init() function
*  @param[in] sen_addr  - 8-byte ROM address of DS18B20
*  @return - None
*/
static void ds18b20_update(mraa_uart_ow_context uart_path, 
                            uint8_t sen_addr[DS18B20_ADDR_LEN]);

/* Function declaration to read analog output voltage corresponding to dust 
*  concentration in air from GP2Y1010AU sensor 
*  @param[in] dust_gpio_path - GPIO instance returned by mraa_gpio_init() function 
*  @param[in] dust_aio_path  - AIO instance returned by mraa_aio_init() function
*  @return - float (Dust concentration in term of voltage)
*/
static float gp2y_read_dust_output_voltage_float(mraa_gpio_context dust_gpio_path, 
                                                   mraa_aio_context dust_aio_path);

/* Function declaration to read humidity as percentage from HSM-20G sensor 
*  @param[in] hsm_aio_path  - AIO instance returned by mraa_aio_init() function
*  @return - float (Humidity in percentage)
*/
static float hsm_read_humidity_float(mraa_aio_context hsm_aio_path);

/* Function declaration to store sensor(s) data to sqlite3 database
*  @param[in] sen_id  - Sensor ID as uint8_t
*  @param[in] sen_val - Sensor value as float
*  @return - uint8_t ( SUCCESS(1), FAIL(0) )
*/
static uint8_t store_data_to_db(uint8_t sen_id, float sen_val);

/******************************************************************************/

int main(int argc, char** argv) 
{
    // UART instance to talk to DS18B20 (1-wire over UART)
    static mraa_uart_ow_context uart_path;
    static mraa_result_t 		mraa_ret_val;
	
    // GPIO instance and pin number to trigger IR LED of dust 
    // sensor (GP2Y1010AU)
    // AIO instance and pin number to read data from dust sensor
    static mraa_gpio_context dust_gpio_path;
    static mraa_aio_context  dust_aio_path;
    static const uint8_t dust_ir_led_pin = 4;
    static const uint8_t dust_aio_in = 0;
	
    // AIO instance and pin number to read humidity from hsm-20g sensor
    static mraa_aio_context  hsm_aio_path;
    static const uint8_t     hsm_aio_in  = 1;
	
    static app_states_t state_index = NONE;
	
    static uint8_t ds18b20_addr[NUM_OF_SENSORS][DS18B20_ADDR_LEN] = {0};
    static float   temp = 0.0;
    static uint8_t sen_count = 0;
    
    static float humidity = 0.0;
    
    static float dust_concentration = 0.0;
    static float acc_dust_concentration = 0.0;
    static const uint8_t dust_sensor_max_sample = 16;
    static uint8_t       dust_sensor_sample_count = 0;
    static uint8_t       dust_data_coll_err = 0;
    
    // We choose '0' as the argument of 'mraa_uart_ow_init()' since we have
    // only singel UART in IoT2020.
    uart_path = mraa_uart_ow_init(0);
    if (NULL == uart_path) 
    {
        printf("mraa_uart_ow_init() failed.\n");
        return 1;
    }
    else
    {
        printf("UART instance created.\n");
    }
	
    mraa_ret_val = mraa_uart_ow_reset(uart_path);
    if (MRAA_SUCCESS == mraa_ret_val) 
    {
        printf("Reset succeeded, device(s) found on bus!\n");
    } 
    else 
    {
        printf("Reset failed, returned %d. No devices on bus?\n", mraa_ret_val);
        
	    mraa_uart_ow_stop(uart_path);
        return 1;
    }

    printf("Seraching for devices.\n");
    
    mraa_ret_val = mraa_uart_ow_rom_search(uart_path, NEW_SEARCH, 
                                                    ds18b20_addr[sen_count++]);
    if (MRAA_ERROR_UART_OW_NO_DEVICES == mraa_ret_val) 
    {
        printf("No devices detected.\n");
        
	    mraa_uart_ow_stop(uart_path);
        return 1;
    }
	
    // Search the other sensors if we have multiple sensor connected on 
    // the same bus.
    if (NUM_OF_SENSORS > 1) 
    {
	    do 
	    {
		    mraa_ret_val = mraa_uart_ow_rom_search(uart_path, CONTINUE_WITH_PREV_SEARCH, 
                                                                 ds18b20_addr[sen_count]);
		    if (MRAA_ERROR_UART_OW_NO_DEVICES == mraa_ret_val) 
		    {
			    printf("Failed to find desired number of sensors on bus.\n");
                
			    mraa_uart_ow_stop(uart_path);
			    return 1;
		    }
	    } while (NUM_OF_SENSORS != (++sen_count));
    }
    if (MRAA_ERROR_UART_OW_DATA_ERROR == mraa_ret_val) 
    {
        printf("Bus or data error.\n");
        
	    mraa_uart_ow_stop(uart_path);
        return 1;
    }
	
    dust_gpio_path = mraa_gpio_init(dust_ir_led_pin);
    if (NULL == dust_gpio_path)
    {
	    printf("Failed to open GPIO instance of pin %d.\n", dust_ir_led_pin);
        
	    mraa_uart_ow_stop(uart_path);
	    return 1;
    }
    else
    {
	    printf("GPIO instance of pin %d created.\n", dust_ir_led_pin);
    }
	
    mraa_ret_val = mraa_gpio_dir(dust_gpio_path, MRAA_GPIO_OUT);
    if (MRAA_SUCCESS == mraa_ret_val) 
    {
	    printf("Config GPIO pin %d as output.\n", dust_ir_led_pin);
    }
    else 
    {
	    printf("Failed to config GPIO pin %d as output.\n", dust_ir_led_pin);
        
	    mraa_gpio_close(dust_gpio_path);
        mraa_uart_ow_stop(uart_path);
	    return 1;
    }
	
    dust_aio_path = mraa_aio_init(dust_aio_in);
    if (NULL == dust_aio_path)
    {
	    printf("Failed to open AIO instance of pin %d.\n", dust_aio_in);
        
	    mraa_gpio_close(dust_gpio_path);
	    mraa_uart_ow_stop(uart_path);
	    return 1;
    }
    else
    {
	    printf("AIO instance of pin %d created.\n", dust_aio_in);
    }
    
    hsm_aio_path = mraa_aio_init(hsm_aio_in);
    if (NULL == hsm_aio_path)
    {
	    printf("Failed to open AIO instance of pin %d.\n", hsm_aio_in);
        
        mraa_gpio_close(dust_gpio_path);
        mraa_aio_close(dust_aio_path);
        mraa_uart_ow_stop(uart_path);
        return 1;
    }
    else
    {
        printf("AIO instance of pin %d created.\n", hsm_aio_in);
    }
	
    // Set the value of sensor counter to 0 in order start new iteration
    sen_count = 0;
    
    for ( ; ; ) 
    {
        switch (state_index) 
        {
            // Nothing to do here, just moving forward
            case NONE:
                state_index = START_CONV;
            break;
			
            case START_CONV:
                ds18b20_update(uart_path, ds18b20_addr[sen_count]);
                state_index = WAIT_TILL_CONV_FINISHED;
            break;
			
            // A delay of 1 sec in order to finish the conversion
            // Maximum conversion time 750 msecs (12-bit resolution)
            case WAIT_TILL_CONV_FINISHED:
                sleep(1);
                state_index = READ_TEMP;
            break;
			
            case READ_TEMP:
                if (SUCCESS == ds18b20_read_temp_float(uart_path, ds18b20_addr[sen_count++], &temp)) 
                {
                    // NOTE: In database we writing fisrt sensor id starts from 1 instead of 0 here
                    if (SUCCESS == store_data_to_db(sen_count, temp)) 
                    {
                        if (sen_count >= NUM_OF_SENSORS) 
                        {
                            // All temperature sensor read, time to collect dust concentration
                            sen_count = 0;
                            state_index = READ_DUST_CONCENTRATION;
                        }
                        else
                        {
                            // Start conversion of next DS18B20 sensor in case of multiple sensors
                            // on the bus
                            state_index = START_CONV;
                        }
                    }
                    else
                    {
                        // Something bad happened, go to default case 
                        printf("Failed to store data of sensor ID=%d.\n", sen_count);
                        state_index = (WAIT + 1);
                    }
                }
                else
                {
                    // Something bad happened, go to default case
                    printf("Error in collecting ds18b20 sensor data.\n");
                    state_index = (WAIT + 1);
                }
            break;
			
            case READ_DUST_CONCENTRATION:
                dust_data_coll_err = 0;
                acc_dust_concentration = 0.0;
                
                // In order to get a stable value from sensor, we are taking reading defined in
                // 'dust_sensor_max_sample' variable. 
                for (dust_sensor_sample_count = 0; dust_sensor_sample_count < dust_sensor_max_sample; 
                                                                            dust_sensor_sample_count++)
                {
                    dust_concentration = gp2y_read_dust_output_voltage_float(dust_gpio_path, 
                                                                                dust_aio_path);
                                                                             
                    if (dust_concentration >= 0.0) 
                    {
                        acc_dust_concentration += dust_concentration;
                    }
                    else
                    {
                        // Something bad happened, go to default case
                        printf("Error in collecting dust sensor data.\n");
                        
                        dust_data_coll_err = 1;
                    }
                }
				
                if (dust_data_coll_err)
                {
                    state_index = (WAIT + 1);
                }
                else
                {
                    acc_dust_concentration = acc_dust_concentration / ((float)dust_sensor_max_sample);
                    
                    // Calculate dust density based on sensor output voltage and put the result
                    // in same variable.
                    // NOTE: Please check the below link understanding the calculation
                    // https://github.com/nazmul21/IoT2020/tree/master/safety%20assistant/documents
                    if (acc_dust_concentration <= 0.6)
                    {
                        // GP2Y1010AU can produce a valid dust concentration value after 0.6
                        // If output voltage is equal or lower than this threshold, then we
                        // define '0.0' as dust concentration.
                        acc_dust_concentration = 0.0;
                    }
                    else if ((acc_dust_concentration > 0.6) && (acc_dust_concentration <= 3.5))
                    {
                        // This equlation is only valid for sensor output voltage in a range of (0 ~ 3.5)
                        acc_dust_concentration = ( acc_dust_concentration - 0.6 ) / 5.8;
                    }
                    else 
                    {
                        // It seems from datasheet, the output voltage of GP2Y1010AU is almost constant                         
                        // i.e. after 3.5V even though the dust concentration is going higher.
                        // So, we cut off the dust concentration limit to '0.6'.
                        acc_dust_concentration = 0.6;
                    }
				
                    // NOTE: I choose (NUM_OF_SENSORS+1) as param 'sen_id', since this is my first 
                    //    	 sensor beyond all the temperature sensor(s). 
                    if (SUCCESS == store_data_to_db((NUM_OF_SENSORS + 1), acc_dust_concentration))
                    {
                        state_index = READ_HUMIDITY;
                    }
                    else
                    {
                        // Something bad happened, go to default case
                        printf("Failed to store data of sensor ID=%d.\n", (NUM_OF_SENSORS + 1));
                        state_index = (WAIT + 1);
                    }
                }
            break;
			
            case READ_HUMIDITY:
                humidity = hsm_read_humidity_float(hsm_aio_path);
				
                // NOTE: second sensor beyond all temperature sensors (NUM_OF_SENSORS+2)
                if (SUCCESS == store_data_to_db((NUM_OF_SENSORS + 2), humidity))
                {
                    state_index = WAIT;
                }
                else
                {
                    // Something bad happened, go to default case
                    printf("Failed to store data of sensor ID=%d.\n", (NUM_OF_SENSORS + 2));
                    state_index = (WAIT + 1);
                }
            break;
			
            case WAIT:
                sleep(60);
                state_index = START_CONV;
            break;
			
            // Application shouldn't reach here. If there is an exception
            // then loop forever here.
            default:
                printf("Something bad happened.\n");
                
                mraa_gpio_close(dust_gpio_path);
                mraa_aio_close(dust_aio_path);
                mraa_aio_close(hsm_aio_path);
                mraa_uart_ow_stop(uart_path);
                while(1);
            break; 
        }
    }
    return 0;
}

/* Definition of read temperature data from DS18B20. 
*/
static uint8_t ds18b20_read_temp_float(mraa_uart_ow_context uart_path, 
									   uint8_t sen_addr[8], float * p_temp) 
{
    assert(NULL != uart_path);
  
    static const uint8_t ds18b20_scratchpad_len = 9;
    uint8_t   ds18b20_scratchpad[ds18b20_scratchpad_len];
     
    #if defined(RUN_TIME_LOG)
        printf("Device Family 0x%02x, ID %02x%02x%02x%02x%02x%02x CRC 0x%02x\n", 
			sen_addr[0], sen_addr[6], sen_addr[5], sen_addr[4], sen_addr[3], 
			sen_addr[2], sen_addr[1], sen_addr[7]);
    #endif
	
    // Issue a scratchpad read command to the specified device
    mraa_uart_ow_command(uart_path, CMD_READ_SCRATCHPAD, sen_addr);
    
    uint8_t i;
    for (i = 0; i < ds18b20_scratchpad_len; i++) 
    {
        ds18b20_scratchpad[i] = (uint8_t)mraa_uart_ow_read_byte(uart_path);
    }
	
    // Calculate the CRC based on value read from scratchpad
    //  Check if the calculated CRC match with the device internal's
    uint8_t crc = mraa_uart_ow_crc8(ds18b20_scratchpad, DS18B20_ADDR_LEN);
    if (crc != ds18b20_scratchpad[8])
    {
        printf("CRC Error.\n");
		
        return FAIL;
    }
	
    // Accumulate the high and low temperature data 
    int16_t temp_adc_val = ( (((int16_t)ds18b20_scratchpad[1]) << 8) | 
                                    ((int16_t)ds18b20_scratchpad[0]) );
	
    // Check if the temperature is negative or not
    // If negative, then convert the 2's complement (negative) to binary (positive) 
    // and return the temperature value
    uint8_t is_temp_negative = (ds18b20_scratchpad[1] & 0x80) ? (1) : (0);
    if (is_temp_negative) 
    {
        temp_adc_val = ( (int16_t)(~temp_adc_val) ) + 1;
        temp_adc_val = -temp_adc_val;
    }
    
    *p_temp = (((float)temp_adc_val) * 0.0625); 
	
    return SUCCESS;
}

static void ds18b20_update(mraa_uart_ow_context uart_path, uint8_t sen_addr[8]) 
{
    assert(NULL != uart_path);
	
    // Issue a start conversion command to the specified device
    mraa_uart_ow_command(uart_path, CMD_START_TEMP_CONV, sen_addr);
}

static float gp2y_read_dust_output_voltage_float(mraa_gpio_context dust_gpio_path, 
                                                    mraa_aio_context dust_aio_path)
{
    assert(NULL != dust_gpio_path);
    assert(NULL != dust_aio_path);
	
    mraa_result_t mraa_ret_val;
    static uint16_t dust_adc_val = 0;
    static float    dust_output_vol = 0.0;
    
    // Turn on IR LED and wait until the hold time of 0.28 msecs, which defined in
    // datasheet
    mraa_ret_val = mraa_gpio_write(dust_gpio_path, 1);
    if (MRAA_SUCCESS == mraa_ret_val) 
    {
        #if defined(RUN_TIME_LOG)
            printf("IR LED turned on.\n");
        #endif
    }
    else 
    {
        printf("Falied to turn on IR LED.\n");
        return 0.0;
    }
    usleep(280);
	
    // Read dust concentration and wait until the rest of pulse 
    // width period of 0.32 ms
    dust_adc_val = mraa_aio_read(dust_aio_path);
    usleep(40);
    
    // Turn off IR LED and wait until the rest of pulse cycle
    // of 10 msecs. i.e (10 - 0.32) msecs = 9680 usecs
    mraa_ret_val = mraa_gpio_write(dust_gpio_path, 0);
    if (MRAA_SUCCESS == mraa_ret_val) 
    {
        #if defined(RUN_TIME_LOG)
            printf("IR LED turned off.\n");
        #endif
    }
    else 
    {
        printf("Falied to turn off IR LED.\n");
        return 0.0;
    }
    usleep(9680);
    
    dust_output_vol = (((float)dust_adc_val/1023.0) * 5.0); 
	
    return dust_output_vol;
}

static float hsm_read_humidity_float(mraa_aio_context hsm_aio_path)
{
    assert(NULL != hsm_aio_path);
    
    static uint16_t hum_adc_val;
    static float humidity = 0.0;
    
    hum_adc_val = mraa_aio_read(hsm_aio_path);
    
    // Calculate the humidity based on analog voltage
    // NOTE: Please check the below link to get details of equiation
    // https://github.com/nazmul21/IoT2020/tree/master/safety%20assistant/documents
    humidity = (((float)hum_adc_val / 1023.0) * 5.0);
    humidity = ((1.253 * humidity * humidity) + 
                        (25.931 * humidity) - 7.542);
    
    return humidity;
}

static uint8_t store_data_to_db(uint8_t sen_id, float sen_val) 
{
    static sqlite3 *p_db_handle;
    static char *p_db_err_msg;
    static char *p_db_query;
    static uint16_t db_ret_val;
    
    db_ret_val = SQLITE_ERROR;
    
    db_ret_val = sqlite3_open(DATABASE_PATH, &p_db_handle);
    if (SQLITE_OK == db_ret_val) 
    {
        #if defined(RUN_TIME_LOG)
            printf("Database opened successfully.\n");
        #endif
    }
    else 
    {
        printf("Database open failed: %s\n", sqlite3_errmsg(p_db_handle));
        sqlite3_close(p_db_handle);
        
        return FAIL;
    }
	
    // Try to allocate a 64 bytes of dynamic memory for query buffer
    p_db_query = (char *) malloc(64);
    if (NULL == p_db_query) 
    {
        printf("malloc() failed to allocate.\n");			
        sqlite3_close(p_db_handle);
        return FAIL;
    }
    else 
    {
        sprintf(p_db_query, "INSERT INTO sensor_data (sen_id, sen_val) values (%d, %0.2f);", 
                                                                             sen_id, sen_val);
		
        #if defined(RUN_TIME_LOG)
            printf("SQLite3 Query - %s\n", p_db_query);
        #endif
		
        db_ret_val = sqlite3_exec(p_db_handle, (const char *)p_db_query, NULL, NULL, 
                                                                        &p_db_err_msg);
        if (SQLITE_OK == db_ret_val) 
        {
            #if defined(RUN_TIME_LOG)
                printf("Data stored to SQLite.\n");
            #endif
        }
        else 
        {
            printf("Falied to store data. Err Msg - %s.\n", p_db_err_msg);
            sqlite3_free(p_db_err_msg);			
            sqlite3_close(p_db_handle);
            return FAIL;
        }
        
        // Set free the allocated memory for the query buffer
        free(p_db_query);
    }
	
    sqlite3_close(p_db_handle);
	
    return SUCCESS;
}
