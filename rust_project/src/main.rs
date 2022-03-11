// Skeleton code for your Rust projects
// I added several comments and annotations to this file.
// _Please_ read them carefully. They are very important.
// The most important comments are all annotated with "NOTE/WARNING:"

// I will grade your code quality primarily on how "idiomatic" your Rust 
// code is, and how well you implemented the "safe unsafety" guidelines.

extern crate libc;
extern crate time;
extern crate ctrlc;
#[macro_use] extern crate simple_error;
extern crate shuteye;
extern crate mmap;
extern crate nix;
extern crate byteorder;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::{fs::OpenOptions, os::unix::fs::OpenOptionsExt};
use std::os::unix::io::AsRawFd;
use std::path::Path;
use std::fs::File;
use std::time::Duration;
use mmap::{MemoryMap, MapOption};
use std::mem::size_of;
use std::thread;
use std::io::{Read, Cursor};
use std::io::{Seek, SeekFrom};
use byteorder::{ReadBytesExt};


#[derive(Copy, Clone)]
#[derive(Debug)]
struct Pixel {
    r: u16,
    g: u16,
    b: u16,
}

struct GPIO {
    gpio_map_: Option<MemoryMap>,
    output_bits_: u32,
    input_bits_: u32,
    slowdown_: u32,                         // Please refer to the GPIO_SetBits and GPIO_ClearBits functions in the reference implementation to see how this is used.
    gpio_port_: *mut u32,                   // A raw pointer that points to the base of the GPIO register file
    gpio_set_bits_: *mut u32,               // A raw pointer that points to the pin output register (see section 2.1 in the assignment)
    gpio_clr_bits_: *mut u32,               // A raw pointer that points to the pin output clear register (see section 2.1)
    gpio_read_bits_: *mut u32,              // A raw pointer that points to the pin level register (see section 2.1)
    row_mask: u32,                      
    bitplane_timings: [u32; COLOR_DEPTH]    
}

// This is a representation of the "raw" image
struct Image {
    width: usize,
    height: usize,
    pixels: Vec<Vec<Pixel>>
}

// This is a representation of the frame we're currently rendering
struct Frame {
    cols: usize,
    img: Image,
    pos: usize,
    pixels: Vec<Vec<Pixel>>
}

// Use this struct to implement high-precision nanosleeps
struct Timer {
    _timemap: Option<MemoryMap>,
    timereg: *mut u32 // a raw pointer to the 1Mhz timer register (see section 2.5 in the assignment)
}
// ============================================================================
// GPIO configuration parameters for the raspberry pi 3
// ============================================================================

const BCM2709_PERI_BASE: u64 = 0x3F000000;
const GPIO_REGISTER_OFFSET: u64 = 0x200000;
const TIMER_REGISTER_OFFSET: u64 = 0x3000;
const REGISTER_BLOCK_SIZE: u64 = 4096;
const ROWS: usize = 16;
const COLUMNS: usize = 32;
const SUB_PANELS_: usize = 2;
const COLOR_DEPTH: usize = 8;
const GAMMA_FACTOR: f32 = 2.2;

const PIN_OE  : u64 = 4;
const PIN_CLK : u64 = 17;
const PIN_LAT : u64 = 21;
const PIN_A   : u64 = 22;
const PIN_B   : u64 = 26;
const PIN_C   : u64 = 27;
const PIN_D   : u64 = 20;
const PIN_E   : u64 = 24;
const PIN_R1  : u64 = 5;
const PIN_G1  : u64 = 13;
const PIN_B1  : u64 = 6;
const PIN_R2  : u64 = 12;
const PIN_G2  : u64 = 16;
const PIN_B2  : u64 = 23;

// Convenience macro for creating bitmasks. See comment above "impl GPIO" below
macro_rules! GPIO_BIT {
    ($bit:expr) => {
        1 << $bit
    };
}

// Use this bitmask for sanity checks
const VALID_BITS: u64 = GPIO_BIT!(PIN_OE) | GPIO_BIT!(PIN_CLK) | GPIO_BIT!(PIN_LAT) |
    GPIO_BIT!(PIN_A)  | GPIO_BIT!(PIN_B)  | GPIO_BIT!(PIN_C)   | GPIO_BIT!(PIN_D)   | GPIO_BIT!(PIN_E) |
    GPIO_BIT!(PIN_R1) | GPIO_BIT!(PIN_G1) | GPIO_BIT!(PIN_B1)  |
    GPIO_BIT!(PIN_R2) | GPIO_BIT!(PIN_G2) | GPIO_BIT!(PIN_B2);

// ============================================================================
// mmap_bcm_register - convenience function used to map the GPIO register block
// ============================================================================

fn mmap_bcm_register(register_offset: usize) -> Option<MemoryMap> {

    let mem_file = 
        match OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(libc::O_SYNC)
        .open("/dev/mem") {
            Err(why) => panic!("couldn't open /dev/mem: {}", why),
            Ok(file) => file
        };

    let mmap_options = &[
        MapOption::MapNonStandardFlags(libc::MAP_SHARED),
        MapOption::MapReadable,
        MapOption::MapWritable,
        MapOption::MapFd(mem_file.as_raw_fd()),
        MapOption::MapOffset(BCM2709_PERI_BASE as usize + register_offset as usize)       
    ];    

    let result = MemoryMap::new(REGISTER_BLOCK_SIZE as usize, mmap_options).unwrap();

    return match result.data().is_null() {
        true => {
            eprintln!("mmap error: {}", std::io::Error::last_os_error());
            eprintln!("Pi3: MMapping from base 0x{:X}, offset 0x{:X}", BCM2709_PERI_BASE, register_offset);
            None
        },
        false => Some(result)
    };
    
    // NOTE/WARNING: When a MemoryMap struct is dropped, the mapped 
    // memory region is automatically unmapped!
}

fn get_correct_rgb_value(srgb_color_value: u16) -> u16 {
    //Normalise the color value
    let normalised_srgb_color_value: f32 = (f32::from(srgb_color_value) / 255.0) as f32;

    //Get the RGB value
    let correction_factor = GAMMA_FACTOR;
    let normalised_rgb_color_value = f32::powf(normalised_srgb_color_value, correction_factor);

    let rgb_color_value = (normalised_rgb_color_value*255.0) as u16;

    rgb_color_value
}

//
// NOTE/WARNING: In many cases, particularly those where you need to set or clear 
// multiple bits at once, it is convenient to store multiple pin numbers in one bit 
// mask value. If you want to simultaneously set PIN_A and PIN_C to high, for example, 
// you should probably create a bit mask with the positions of PIN_A and PIN_C set to 1, 
// and all other positions set to 0. You can do this using the GPIO_BIT! macro.
//
// In this example, you would do something like:
//     let pin_mask = GPIO_BIT!(PIN_A) | GPIO_BIT!(PIN_C);
//     io.set_bits(pin_mask);
//
impl GPIO {

    //
    // configures pin number @pin_num as an output pin by writing to the 
    // appropriate Function Select register (see section 2.1).
    // 
    // NOTE/WARNING: This method configures one pin at a time. The @pin_num argument 
    // that is expected here is really a pin number and not a bitmask!
    //
    // Doing something like:
    //     io.configure_output_pin(VALID_BITS);
    // Would be WRONG! This call would make the program crash.
    //
    // Doing something like:
    //     if GPIO_BIT!(PIN_A) & VALID_BITS {
    //         io.configure_output_pin(PIN_A);
    //     }
    // Would be OK!
    //
    fn configure_output_pin(self: &mut GPIO, pin_num: u64) {
        let register_num = (pin_num / 10) as isize;
        let register_ref = unsafe { self.gpio_port_.offset(register_num) };
        // NOTE/WARNING: When reading from or writing to MMIO memory regions, you MUST 
        // use the std::ptr::read_volatile and std::ptr::write_volatile functions
        let current_val = unsafe { std::ptr::read_volatile(register_ref) };
        // the bit range within the register is [(pin_num % 10) * 3 .. (pin_num % 10) * 3 + 2]
        // we need to set these bits to 001
        let new_val = (current_val & !(7 << ((pin_num % 10)*3))) | (1 << ((pin_num % 10)*3));
        // NOTE/WARNING: When reading from or writing to MMIO memory regions, you MUST 
        // use the std::ptr::read_volatile and std::ptr::write_volatile functions
        unsafe { std::ptr::write_volatile(register_ref, new_val) };
    }

    fn init_outputs(self: &mut GPIO, outputs: u32) -> u32 {
        // TODO: Implement this yourself. Note: this function expects 
        // a bitmask as the @outputs argument
        
        //no need to check for null struct: can't exist in Rust

        //both of these HAVE to be true at the same time
        //raw pointer CAN be null though. need to check to make sure it isn't
        //make sure it's not a default 0 either -> this would mean incorrect init
        assert!(!self.gpio_port_.is_null() && !(self.gpio_port_ == 0 as *mut u32));

        //set GPIO4 && GPIO18 both as input
        //necessary to make sure both adafruit-hat && adafruit-hat-pwm can be used
        //NOT NEEDED HERE!!


        //make all pins defined from line 78 usable as output pins
        //use the function "configure_output_pin" hereabove for that!
        //Also configure all outputs
        for bit in 0..28 {
            if outputs & (1 << bit) != 0 {
                self.configure_output_pin(bit)
            }
        }

        outputs
    }

    fn set_bits(self: &mut GPIO, value: u32) {
        // TODO: Implement this yourself. Remember to take the slowdown_ value into account!
        // This function expects a bitmask as the @value argument
        if value == 0 {
            return
        } else {
            unsafe {
                std::ptr::write_volatile(self.gpio_set_bits_, value);
                for _ in 0..self.slowdown_+1 {
                    std::ptr::write_volatile(self.gpio_set_bits_, value);
                }
            }
        }
    }

    fn clear_bits(self: &mut GPIO, value: u32) {        
        // TODO: Implement this yourself. Remember to take the slowdown_ value into account!
        // This function expects a bitmask as the @value argument
        if value == 0 {
            return
        } else {
            unsafe {
                std::ptr::write_volatile(self.gpio_clr_bits_, value);
                for _ in 0..self.slowdown_+1 {
                    std::ptr::write_volatile(self.gpio_clr_bits_, value);
                }
            }
        }
    }

    // Write all the bits of @value that also appear in @mask. Leave the rest untouched.
    // @value and @mask are bitmasks
    fn write_masked_bits( 
        self: &mut GPIO,
        value: u32,
        mask: u32
    ) {
        // TODO: Implement this yourself.
        self.clear_bits(!value & mask);
        self.set_bits(value & mask);
    }

    fn new(slowdown: u32) -> GPIO {

        // Map the GPIO register file. See section 2.1 in the assignment for details
        let map = mmap_bcm_register(GPIO_REGISTER_OFFSET as usize);
        
        // Initialize the GPIO struct with default values
        let mut io: GPIO = GPIO {
            gpio_map_: None,
            output_bits_: 0,
            input_bits_: 0,
            slowdown_: slowdown,
            gpio_port_: 0 as *mut u32,
            gpio_set_bits_: 0 as *mut u32,
            gpio_clr_bits_: 0 as *mut u32,
            gpio_read_bits_: 0 as *mut u32,
            row_mask: 0,
            bitplane_timings: [0; COLOR_DEPTH]  
        };

        match &map {
            Some(m) => {
                unsafe {
                    io.gpio_port_ = m.data() as *mut u32;
                    // TODO: Calculate the correct values of the other raw pointers here.
                    // You should use the offset() method on the gpio_port_ pointer.
                    // Keep in mind that Rust raw pointer arithmetic works exactly like
                    // C pointer arithmetic. See the course slides for details
                    let set_bits_offset = (0x1C / size_of::<u32>()) as isize;
                    let clr_bits_offset = (0x28 / size_of::<u32>()) as isize;
                    let read_bits_offset = (0x34 / size_of::<u32>()) as isize;
                    io.gpio_set_bits_ = io.gpio_port_.offset(set_bits_offset) as *mut u32;
                    io.gpio_clr_bits_ = io.gpio_port_.offset(clr_bits_offset) as *mut u32;
                    io.gpio_read_bits_ = io.gpio_port_.offset(read_bits_offset) as *mut u32;
                }
                
                // TODO: Implement this yourself.
                //Rowmask setup
                io.row_mask =  GPIO_BIT!(PIN_A);
                if ROWS / SUB_PANELS_ > 2 { io.row_mask |= GPIO_BIT!(PIN_B) };
                if ROWS / SUB_PANELS_ > 4 { io.row_mask |= GPIO_BIT!(PIN_C) };
                if ROWS / SUB_PANELS_ > 8 { io.row_mask |= GPIO_BIT!(PIN_D) };
                if ROWS / SUB_PANELS_ > 16 { io.row_mask |= GPIO_BIT!(PIN_E) };

                //Set output pins
                io.output_bits_ = io.init_outputs(VALID_BITS as u32);

                //Set the bitplane timings
                let mut timing_ns = 2000;
                for i in 0..COLOR_DEPTH {
                    io.bitplane_timings[i] = timing_ns;
                    timing_ns *= 2;
                }
            },
            None => {}
        }

        io.gpio_map_ = map;
        io
    }
    
    // Calculates the pins we must activate to push the address of the specified double_row
    fn get_row_bits(self: &GPIO, double_row: u8) -> u32 {
        // TODO: Implement this yourself.
        let mut rowbits = 0;
        if double_row & (1 << 0) !=0 {rowbits |= GPIO_BIT!(PIN_A)};
        if double_row & (1 << 1) !=0 {rowbits |= GPIO_BIT!(PIN_B)}; 
        if double_row & (1 << 2) !=0 {rowbits |= GPIO_BIT!(PIN_C)}; 
        if double_row & (1 << 3) !=0 {rowbits |= GPIO_BIT!(PIN_D)};
        if double_row & (1 << 4) !=0 {rowbits |= GPIO_BIT!(PIN_E)};

        rowbits as u32
    }
}

impl Timer {
    // Reads from the 1Mhz timer register (see Section 2.5 in the assignment)
    unsafe fn read(self: &Timer) -> u32 {
        // TODO: Implement this yourself.
        let time_value : u32 = std::ptr::read_volatile(self.timereg);

        time_value
    }

    fn new() -> Timer {
        // TODO: Implement this yourself.
        let map = mmap_bcm_register(TIMER_REGISTER_OFFSET as usize);
        
        // Initialize the Timer struct with default values
        let mut timer: Timer = Timer {
            _timemap: None,
            timereg: 0 as *mut u32 
        };

        match &map {
            Some(m) => {
                unsafe {
                    let base = m.data() as *mut u32;
                    let timereg_offset = (0x4 / size_of::<u32>()) as isize;
                    timer.timereg = base.offset(timereg_offset) as *mut u32;
                }
            },
            None => {}
        }

        timer._timemap = map;
        timer
    }

    // High-precision sleep function (see section 2.5 in the assignment)
    // NOTE/WARNING: Since the raspberry pi's timer frequency is only 1Mhz, 
    // you cannot reach full nanosecond precision here. You will have to think
    // about how you can approximate the desired precision. Obviously, there is
    // no perfect solution here.
    fn nanosleep(self: &Timer, nanos: u32) {
        // TODO: Implement this yourself.
        //Convert the nano seconds to a duratinon
        let duration: Duration =  Duration::new(0, nanos);
        
        //Read the start time
        let start_time: u32;
        unsafe {
            start_time = self.read();
        }
        

        //Sleep usually adds 66000 extra nanoseconds, so subtract this
        //66000 nanoseconds is also the minimum amount of sleep
        if duration.as_nanos() >= 66000 {
            //Sleep for the specified amount of microseconds
            let updated_duration = duration.checked_sub(Duration::new(0, 66000));
            thread::sleep(updated_duration.unwrap());

        } 
        
        //Set the elapsed time
        let mut elapsed_time: u32 = self.get_elapsed_time(start_time);
        

        //if the thread has slept for a long enough time
        if elapsed_time >= duration.as_micros() as u32 {return}
        else {
            // spin the rest of the duration
            while elapsed_time < duration.as_micros() as u32 {
                thread::yield_now();

                //Read the new elapsed time
                elapsed_time = self.get_elapsed_time(start_time);
            }
        }
        
        /*
        //Read the end time
        let end_time: u32;
        unsafe {
            end_time = self.read();
        }
        let sleep_time: u64 = (end_time - start_time).into();
        println!("sleeptime {}", sleep_time );
        println!("wanted Sleep {}", duration.as_micros());
        println!();
        */
        
        
    }

    fn get_elapsed_time(self: &Timer, start_time: u32) -> u32 {
        let mut elapsed_time: u32 = 0;

        //Get the current time
        let current_time: u32 = unsafe {
            self.read()
        };

        //Test if there was a timer register overflow
        let overflowed: bool = current_time < start_time;

        if overflowed {
            //Calculate the time from the start time to the max value
            elapsed_time += u32::MAX - start_time;

            //Combine with the current time
            elapsed_time += current_time;
        } else {
            elapsed_time += current_time - start_time;
        }


        elapsed_time
    }
}

// TODO: Implement your frame calculation/updating logic here.
// The Frame should contain the pixels that are currently shown
// on the LED board. In most cases, the Frame will have less pixels
// than the input Image!
impl Frame {
    fn new(image: Image) -> Frame {
        //Create an emptyPixel
        let empty_pixel: Pixel = Pixel {
            r: 0,
            g: 0,
            b: 0
        };

        //Initialize a Frame object with nullPixels
        let mut frame: Frame = Frame {
            cols: image.pixels[0].len(),
            img: image,
            pos: 0,
            pixels: vec![vec![empty_pixel; COLUMNS]; ROWS]

        };

        //this makes sure to only interate where there are non-null values. Otherwise errors are possible
        //any pixel without image in it displays (0,0,0)

        let rows = std::cmp::min(frame.img.pixels.len(), ROWS);
        let cols = std::cmp::min(frame.img.pixels[0].len(), COLUMNS);

        for i in 0..rows{
            for j in 0..cols{
                let red = frame.img.pixels[i][j].r;
                let green = frame.img.pixels[i][j].g;
                let blue = frame.img.pixels[i][j].b;

                let pix: Pixel = Pixel {
                    r: red,
                    g: green,
                    b: blue
                };

                frame.pixels[i][j] = pix;
            }
        }

        if frame.cols < COLUMNS{
            frame.pos = frame.cols;
        } else {
            frame.pos = COLUMNS;
        }

        frame
    }

    //consider pos to be the index of the column of the image that is currently displayed rightmost in frame;
    //  this way we can just first move every pixel over in the frame by one column to the left
    //  and then load in column(++pos) in the rightmost frame-column.
    fn render_next(&mut self) {
        // if the image is not wide enough to fill all columns, detect this & fill (0,0,0) pixels
        if self.cols < COLUMNS {
            //now we know that the image is not wide enough to fill all columns,
            //even so, move all current pixels over one to the left, but keep notice on whether or not it's a nullpixel row
            self.pos = (self.pos + 1) % COLUMNS;
            for row in 0..self.pixels.len() {
                for column in 0..self.pixels[0].len() -1{
                    self.pixels[row][column] = self.pixels[row][column+1];
                }
            }

            //next we should detect whether or not this is a null-pixel column.
            if self.pos >= self.cols {
                //this is a null-pixel iteration
                for row in 0..self.pixels.len() {
                    self.pixels[row][COLUMNS-1] = Pixel {
                        r: 0,
                        g: 0,
                        b: 0
                    };
                }

            } else {
                //there are pixels to load from the image
                for row in 0..self.pixels.len() {
                    let red = self.img.pixels[row][self.pos].r;
                    let green = self.img.pixels[row][self.pos].g;
                    let blue = self.img.pixels[row][self.pos].b;

                    let pix: Pixel = Pixel {
                        r: red,
                        g: green,
                        b: blue
                    };

                    self.pixels[row][COLUMNS-1] = pix;
                }
            }
        } else {
            self.pos = (self.pos + 1) % self.cols;
            //first move all columns over to the left by one
            for row in 0..self.pixels.len() {
                for column in 0..self.pixels[0].len() -1{
                    self.pixels[row][column] = self.pixels[row][column+1];
                }
            }
            //load in the new pixels
            for row in 0..self.pixels.len() {
                let red = self.img.pixels[row][self.pos].r;
                let green = self.img.pixels[row][self.pos].g;
                let blue = self.img.pixels[row][self.pos].b;

                let pix: Pixel = Pixel {
                    r: red,
                    g: green,
                    b: blue
                };

                self.pixels[row][COLUMNS-1] = pix;
            }
        }
    }

}

/*
// You do not need to add support for any formats other than P6
// You may assume that the max_color value is always 255, but you should add sanity checks
// to safely reject files with other max_color values
*/

impl Image {
    fn new(width: usize, height: usize, pixels: Vec<Vec<Pixel>>) -> Image {
        //Initialize an Image object
        let image: Image = Image {
            width: width,
            height: height,
            pixels: pixels

        //resize here!
            //strategy: shrink image tot de height de hoogte (16px) van de RPI matchet
                //de lengte zal variabel zijn. Naar alle waarschijnlijkheid is dan de breedte nog steeds >= 32px, anders kan er niet echt gescrolled worden
            //nadien kunnen we de render_next gebruiken om te scrollen over de lengte (lange as):)
            //Lanczos Filtering blijkt zeer goed, maar vond enkel gebrekkige documentatie :/

            //in plaats daarvan werken we met bilinear
            //https://stackoverflow.com/questions/3086770/what-algorithms-to-use-for-image-downsizing
            //blijkbaar is het meer efficient om eerst tot ongeveer de correcte dimensies te downsizen door halvering
        };

        image
    }

    fn get_downscaled_img(pixels: &Vec<Vec<Pixel>>) -> Image {
        println!();  
        let empty_pixel: Pixel = Pixel {
            r: 0,
            g: 0,
            b: 0
        };

        match pixels.len() {
            0 => panic!("empty image received?"),
            _ => ()
        }

        let mut img = vec![vec![empty_pixel; pixels[0].len()]; pixels.len()];
        for i in 0..pixels.len(){
            for j in 0..pixels[0].len(){
                let red = pixels[i][j].r;
                let green = pixels[i][j].g;
                let blue = pixels[i][j].b;

                let pix: Pixel = Pixel{
                    r: red,
                    g: green,
                    b: blue
                };

                img[i][j] = pix;
            }
        }

        if img.len() > ROWS{

            // /2 downscaling was recommended for image quality
            while img.len() >= ROWS*2 {
                println!("resizing [{}, {}] to [{}, {}]", img.len(), img[0].len(), img.len()/2, img[0].len()/2);
                let mut new_img = vec![vec![empty_pixel; img[0].len()/2]; img.len()/2];

                for i in 0..new_img.len(){
                    for j in 0..new_img[0].len(){
                        let red = (img[2*i][2*j].r + img[2*i+1][2*j].r + img[2*i][2*j+1].r + img[2*i+1][2*j+1].r)/4;
                        let green = (img[2*i][2*j].g + img[2*i+1][2*j].g + img[2*i][2*j+1].g + img[2*i+1][2*j+1].g)/4;
                        let blue = (img[2*i][2*j].b + img[2*i+1][2*j].b + img[2*i][2*j+1].b + img[2*i+1][2*j+1].b)/4;

                        let pixel: Pixel = Pixel {
                            r: red,
                            g: green,
                            b: blue
                        };

                        new_img[i][j] = pixel;
                    }
                }
                img = new_img;
            }
            
            let factor = (100*ROWS)/(img.len());
            let newcolsnr:usize = (img[0].len() as f32 * ((factor as f32)/(100.0))) as usize;

            println!();
            println!("/2 downscaling finished. Height: {}, Width: {}", img.len(), img[0].len());
            println!();
            println!("starting downscaling to {}% current size", factor);
            println!("new image dimensions calculated. Height: {}, Width: {}.", ROWS, newcolsnr);

            let mut new_img = vec![vec![empty_pixel; newcolsnr]; 16];

            println!("final size: Height: {}, Width: {}", new_img.len(), new_img[0].len());

            // START BILINEAR -- sadly this ruins any gains to image quality by /2 downscaling :(
            for i in 0..new_img.len(){
                for j in 0..new_img[0].len(){
                    //calculate positions in old img
                    let first_index:usize = ((i as f32 * img.len() as f32)/(new_img.len() as f32)).floor() as usize;
                    let second_index:usize = ((j as f32 * img[0].len() as f32)/(new_img[0].len() as f32)).floor() as usize;

                    //calculate ratios
                    let y_ratio = ((i as f32 * img.len() as f32)/(new_img.len() as f32)) - first_index as f32;
                    let x_ratio = ((j as f32 * img[0].len() as f32)/(new_img[0].len() as f32)) - second_index as f32;

                    //if in doubt, first_index should go to img.len()-2 and second_index to img[0].len()-2; ratios should be 0 <= ratio < 1
                    //println!("(i,j): ({},{}) -> first index: {}, second index: {}",i, j, first_index, second_index);
                    //println!("ratios: x_ratio = {}, y_ratio = {}", x_ratio, y_ratio);

                    //now calculate the intermediate rgb values on x-axis using the correct distance values
                    //upper intermediate
                    let upper_pixel: Pixel = Pixel {
                        r: (img[first_index][second_index].r as f32 * (1 as f32 - x_ratio) + img[first_index][second_index+1].r as f32 * x_ratio) as u16,
                        g: (img[first_index][second_index].g as f32 * (1 as f32 - x_ratio) + img[first_index][second_index+1].g as f32 * x_ratio) as u16,
                        b: (img[first_index][second_index].b as f32 * (1 as f32 - x_ratio) + img[first_index][second_index+1].b as f32 * x_ratio) as u16
                    };

                    let lower_pixel: Pixel = Pixel {
                        r: (img[first_index+1][second_index].r as f32 * (1 as f32 - x_ratio) + img[first_index + 1][second_index+1].r as f32 * x_ratio) as u16,
                        g: (img[first_index+1][second_index].g as f32 * (1 as f32 - x_ratio) + img[first_index + 1][second_index+1].g as f32 * x_ratio) as u16,
                        b: (img[first_index+1][second_index].b as f32 * (1 as f32 - x_ratio) + img[first_index + 1][second_index+1].b as f32 * x_ratio) as u16
                    };

                    //now calculate the final pixel value using the y_ratio
                    let final_pixel: Pixel = Pixel {
                        r: (upper_pixel.r as f32 * (1 as f32 - y_ratio) + lower_pixel.r as f32 * y_ratio) as u16,
                        g: (upper_pixel.g as f32 * (1 as f32 - y_ratio) + lower_pixel.g as f32 * y_ratio) as u16,
                        b: (upper_pixel.b as f32 * (1 as f32 - y_ratio) + lower_pixel.b as f32 * y_ratio) as u16
                    };

                    new_img[i][j] = final_pixel;
                }
            }
            // END BILINEAR

            let downscaled_img: Image = Image {
                height: new_img.len(),
                width: new_img[0].len(),
                pixels: new_img
            };

            downscaled_img
        } else {
            let downscaled_img: Image = Image {
                height: img.len(),
                width: img[0].len(),
                pixels: img
            };

            downscaled_img
        }
    }

    fn get_corrected_rgb_image(rgb_image: &Image) -> Image {
        let empty_pixel: Pixel = Pixel {
            r: 0,
            g: 0,
            b: 0
        };

        //Initialise a new vector using empty pixels 
        let mut pixel_matrix: Vec<Vec<Pixel>> = vec![vec![empty_pixel; rgb_image.width]; rgb_image.height];


        //Convert the current sRGB pixels to RGB pixels
        for row_index in 0..rgb_image.height {
            for column_index in 0..rgb_image.width {
                pixel_matrix[row_index][column_index] = Pixel::get_corrected_rgb_pixel(&rgb_image.pixels[row_index][column_index]);
            }
        }

        //Create a new image
        let corrected_rgb_image: Image = Image::new(rgb_image.width, rgb_image.width, pixel_matrix);

        corrected_rgb_image
    }
}

impl Pixel {
    fn get_corrected_rgb_pixel(rgb_pixel: &Pixel) -> Pixel {
        //Create a new sRGB pixel
        let corrected_rgb_pixel = Pixel {
            r: get_correct_rgb_value(rgb_pixel.r),
            g: get_correct_rgb_value(rgb_pixel.g),
            b: get_correct_rgb_value(rgb_pixel.b)
        };

        corrected_rgb_pixel
    }
}


fn decode_ppm_image(cursor: &mut Cursor<Vec<u8>>) -> Result<Image, std::io::Error> {
    //Create a new image object
    let width = 0;
    let height = 0;
    let pixels = vec![];
    let mut image = Image::new(width, height, pixels);

    //Read the image headers
    let mut header: [u8;2]=[0;2];
    cursor.read(&mut header)?; //Read the first two bytes
    match &header{ 
        b"P6" => println!("P6 image found!"),
        _ => panic!("Not an P6 image")
    }

    //Set the image with and height
    image.width = read_number(cursor)?;
    image.height = read_number(cursor)?;
    let colour_range = read_number(cursor)?;

    match colour_range{
        255 => (),
        _ => panic!("unexpected colour range: {}", colour_range)
    }
    //assert!(colour_range != 255);

    println!("image dimensions: width {:1}, height {:2}, colour range {:3}", image.width, image.height, colour_range);
    //For possible whitespaces after header
    consume_whitespaces(cursor)?;

    //Read the image body
    for _ in 0.. image.height{
        let mut row = Vec::new();
        for _ in 0..image.width{
            let red = cursor.read_u8().unwrap(); //::<BigEndian>().unwrap();
            let green = cursor.read_u8().unwrap(); //::<BigEndian>().unwrap();
            let blue = cursor.read_u8().unwrap(); //::<BigEndian>().unwrap();
            
            //println!("{:?}", (red, green, blue));
            row.push(Pixel {
                r: red as u16,
                g: green as u16,
                b: blue as u16
            });
        }
        image.pixels.push(row);
    }

    Ok(image)
}

fn read_number(cursor: &mut Cursor<Vec<u8>>)-> Result<usize,std::io::Error>{
    consume_whitespaces(cursor)?;

    let mut buff: [u8;1] = [0];
    let mut v = Vec::new();

    loop{
        cursor.read(& mut buff)?;
        match buff[0]{
            b'0'..= b'9' => v.push(buff[0]),
            b' ' | b'\n' | b'\r' | b'\t' => break,
            _ => panic!("Not a valid image")
        }
    }

    let num_str: &str = std::str::from_utf8(&v).unwrap(); // unwrap gaat ok value er uit halen als het ok is, panic als het niet ok is
    let num = num_str.parse::<usize>().unwrap(); // unwrap dient voor errors

    Ok(num)
}


fn consume_whitespaces (cursor: &mut Cursor<Vec<u8>>)-> Result<(),std::io::Error>{ //Result<() : de lege haakjes betekend  niks returnen
    let mut buff: [u8;1] = [0];

    loop{
        cursor.read(& mut buff)?;
        match buff[0]{
            b' ' | b'\n' | b'\r' | b'\t' => (),
            b'#' => {
                consume_comment_line(cursor)?;
            }
            _ => { // je zit eigenlijk al te ver nu !!! zet cursor 1 terug
                cursor.seek(SeekFrom::Current(-1))?;
                break;
            }
        }
    }
    Ok(())
}

fn consume_comment_line (cursor: &mut Cursor<Vec<u8>>)-> Result<(),std::io::Error>{ //Result<() : de lege haakjes betekend  niks returnen
    let mut buff: [u8;1] = [0];

    //skip the entire comment line
    loop {
        cursor.read(& mut buff)?;
        match buff[0]{
            b'\n' | b'\r' => {
                break;
            },
            _ => ()
        }
    }   
    
    Ok(())
}


fn get_plane_bits(pixel_top: &Pixel, pixel_bottom: &Pixel, plane: usize) -> u32 {
    let mut pin_values = 0;
    if (pixel_top.r & (1 << plane))!=0 {pin_values |= GPIO_BIT!(PIN_R1)}
    if (pixel_top.g & (1 << plane))!=0 {pin_values |= GPIO_BIT!(PIN_G1)}; 
    if (pixel_top.b & (1 << plane))!=0 {pin_values |= GPIO_BIT!(PIN_B1)}; 
    if (pixel_bottom.r & (1 << plane))!=0 {pin_values |= GPIO_BIT!(PIN_R2)}; 
    if (pixel_bottom.g & (1 << plane))!=0 {pin_values |= GPIO_BIT!(PIN_G2)}; 
    if (pixel_bottom.b & (1 << plane))!=0 {pin_values |= GPIO_BIT!(PIN_B2)}; 
    
    pin_values as u32
}


pub fn main() {
    let args : Vec<String> = std::env::args().collect();
    let interrupt_received = Arc::new(AtomicBool::new(false));

    // sanity checks
    if nix::unistd::Uid::current().is_root() == false {
        eprintln!("Must run as root to be able to access /dev/mem\nPrepend \'sudo\' to the command");
        std::process::exit(1);
    } else if args.len() < 2 {
        eprintln!("Syntax: {:?} [image]", args[0]);
        std::process::exit(1);
    }
    
    // can be tested cargo build && sudo ./target/debug/rust_project kuleuven_logo.ppm (scroll speed [f32])
    let path = Path::new(&args[1]);
    let mut scroll_slowdown:f32 = 1.0;
    if args.len() == 3 {
        match args[2].parse::<f32>(){
            Ok(n) => scroll_slowdown = n,
            Err(_e) => panic!("Could not parse scroll_speed! Make sure it's a valid number")
        }
    } else {
        println!("Defaulting to scroll-slowdown == 1");
    }
    let display = path.display();

    //Open the file
    let mut file = match File::open(&path) {
        Err(why) => panic!("Could not open file: {} (Reason: {})", 
            display, why),
        Ok(file) => file
    };

    // read the full file into memory. panic on failure
    let mut raw_file = Vec::new();
    file.read_to_end(&mut raw_file).unwrap();

    // construct a cursor so we can seek in the raw buffer
    let mut cursor = Cursor::new(raw_file);

    let image = match decode_ppm_image(&mut cursor) {
        Ok(img) => img,
        Err(why) => panic!("Could not parse PPM file - Desc: {}", why),
    };

    //downscale the img
    let downscaled_img = Image::get_downscaled_img(&image.pixels);

    //Get an sRGB colorspace image using the downscaled image
    let corrected_rgb_image_for_frame = Image::get_corrected_rgb_image(&downscaled_img);

    //Create a new Frame
    let mut frame: Frame = Frame::new(corrected_rgb_image_for_frame);
    frame.render_next();


    // TODO: Initialize the GPIO struct and the Timer struct
    let mut gpio = GPIO::new(0);
    let timer: Timer = Timer::new();

    //Initialise the last render time
    let mut last_render_time: u32 = unsafe {
        timer.read()
    };


    // This code sets up a CTRL-C handler that writes "true" to the 
    // interrupt_received bool.
    let int_recv = interrupt_received.clone();
    ctrlc::set_handler(move || {
        int_recv.store(true, Ordering::SeqCst);
    }).unwrap();
    
    let color_mask = GPIO_BIT!(PIN_R1) | GPIO_BIT!(PIN_G1) | GPIO_BIT!(PIN_B1) | GPIO_BIT!(PIN_R2) | GPIO_BIT!(PIN_G2) | GPIO_BIT!(PIN_B2);
    let color_and_clk_mask = color_mask | GPIO_BIT!(PIN_CLK);
    
    //Render loop
    while interrupt_received.load(Ordering::SeqCst) == false {
        // TODO: Implement your rendering loop here
        for double_row in 0..ROWS/2 {
            for bit in 0..COLOR_DEPTH {
                for column in 0..COLUMNS {
                    let pixel_top = &frame.pixels[double_row][column];
                    let pixel_bottom = &frame.pixels[ROWS/2 + double_row][column];
                    gpio.write_masked_bits(get_plane_bits(pixel_top, pixel_bottom, bit), color_and_clk_mask);
                    gpio.set_bits(GPIO_BIT!(PIN_CLK));
                }
                //reset clock and colors
                gpio.clear_bits(color_and_clk_mask);

                //Set the row address bits
                gpio.write_masked_bits(gpio.get_row_bits(double_row as u8), gpio.row_mask);

                //Toggle the lAT pin to strobe the new double row
                gpio.set_bits(GPIO_BIT!(PIN_LAT));
                gpio.clear_bits(GPIO_BIT!(PIN_LAT));

                //Activate the leds for the specified amount of time
                //Note: Clearing the OE pin activates the leds
                gpio.clear_bits(GPIO_BIT!(PIN_OE));

                //Sleep
                timer.nanosleep(gpio.bitplane_timings[bit]);

                gpio.set_bits(GPIO_BIT!(PIN_OE));
            }
        }

        if timer.get_elapsed_time(last_render_time) >= (25000.0 * scroll_slowdown) as u32 {
            frame.render_next();
            last_render_time = unsafe {
                timer.read()
            };
        }
    }

    println!("Exiting.");
    if interrupt_received.load(Ordering::SeqCst) == true {
        println!("Received CTRL-C");
    } else {
        println!("Timeout reached");
    }
    
    // TODO: You may want to reset the board here (i.e., disable all LEDs)
    
}