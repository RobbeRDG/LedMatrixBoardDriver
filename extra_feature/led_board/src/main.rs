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
use std::error::Error;
use std::os::unix::io::AsRawFd;
use std::path::Path;
use std::io::prelude::*;
use std::fs::File;
use std::time::{Instant, Duration};
use shuteye::sleep;
use mmap::{MemoryMap, MapOption};
use std::ptr;
use std::mem::size_of;
use std::thread;
use std::io::{Read, Cursor};
use std::io::{Seek, SeekFrom};
use byteorder::{ReadBytesExt, BigEndian};
use std::fs;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     CONSTANTS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
const ROWS: usize = 16;
const COLUMNS: usize = 32;
const SUB_PANELS_: usize = 2;
const COLOR_DEPTH: usize = 8;
const HEIGHT: usize = ROWS;
const GAMMA_FACTOR: f32 = 2.2;

const BCM2709_PERI_BASE: u64 = 0x3F000000;
const GPIO_REGISTER_OFFSET: u64 = 0x200000;
const TIMER_REGISTER_OFFSET: u64 = 0x3000;
const REGISTER_BLOCK_SIZE: u64 = 4096;

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





///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

fn sRGB_to_RGB(RGB_color_value: u16) -> u16 {
    //Normalise the color value
    let normalised_RGB_color_value = (RGB_color_value/255) as f32;

    //Get the sRGB value
    let correction_factor = GAMMA_FACTOR;
    let normalised_sRGB_color_value = f32::powf(normalised_RGB_color_value, correction_factor);

    let sRGB_color_value = (normalised_sRGB_color_value*255.0) as u16;

    sRGB_color_value
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

fn get_pixelvalues_from_color(color: &str) -> Pixel {
    let pixelvalue: Pixel = match color {
        "red" => Pixel {
                r: 255,
                g: 0,
                b: 0
            },
        "green" => Pixel {
            r: 0,
            g: 255,
            b: 0
        },
        "blue" => Pixel {
            r: 0,
            g: 0,
            b: 255
        },
        "black" => Pixel {
            r: 0,
            g: 0,
            b: 0
        },
        "white" => Pixel {
            r: 255,
            g: 255,
            b: 255
        },
        "yellow" => Pixel {
            r: 255,
            g: 255,
            b: 0
        },
        "pink" => Pixel {
            r: 255,
            g: 0,
            b: 255
        },
        _ => panic!("The chosen pixel color is not available")
    };

    pixelvalue
} 

fn get_letter_matrix_for_char(letter: char, foreground_pixel: Pixel, background_pixel: Pixel) -> LetterMatrix {
    //Get the text file containing the matrix for the letter
    println!("looking for {}_{}.txt", letter, HEIGHT);
    let file_path = format!("/home/pi/Documents/veilige_software/project/veiligesoftware-20202021-groep6/extra_feature/led_board/letter_matrices/{}_{}.txt", letter, HEIGHT);
    let matrix = fs::read_to_string(file_path).expect(&(format!("Unable to read file for letter '{}'", letter)[..]));

    //Split the letter matrix string in the different rows
    let mut matrix_rows: Vec<&str> = matrix.split(",").collect();

    //Truncate the heigth to fit the led matrix
    matrix_rows.truncate(ROWS);

    //Calculate the height and width of the matrix
    let height = matrix_rows.len();
    let width = matrix_rows[0].chars().count();

    //For each row entry, generate a pixel and place them in the vector
    let mut pixel_matrix = vec![vec![]];
    for row in 0..height {
        //Get the chars for the current row
        let row_chars: Vec<char> = matrix_rows[row].chars().collect();

        //generate the vector of the row of pixels
        let mut pixel_row: Vec<Pixel> = vec![];
        for row_char in row_chars {
            pixel_row.push(match row_char {
                '1' => foreground_pixel,
                '0' => background_pixel,
                _ => panic!("undefined char in letter matrix of letter {}", letter)
            });

        }

        //Add the row of pixels to the matrix
        pixel_matrix.push(pixel_row);
    }
   
    let letter_matrix: LetterMatrix = LetterMatrix::new(width, height, pixel_matrix);

    letter_matrix
}


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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     STRUCTS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Copy, Clone)]
struct Pixel {
    r: u16,
    g: u16,
    b: u16,
}

// This is a representation of the "raw" image
struct MessageMatrix {
    width: usize,
    height: usize,
    pixels: Vec<Vec<Pixel>>
}

struct LetterMatrix {
    width: usize,
    height: usize,
    pixels: Vec<Vec<Pixel>>
}

// This is a representation of the frame we're currently rendering
struct Frame {
    pos: usize,
    cols: usize,
    slowdown: usize,
    sd_i: usize,
    img: Vec<Vec<Pixel>>,
    pixels: Vec<Vec<Pixel>>
}

// Use this struct to implement high-precision nanosleeps
struct Timer {
    _timemap: Option<MemoryMap>,
    timereg: *mut u32 // a raw pointer to the 1Mhz timer register (see section 2.5 in the assignment)
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

    fn init_outputs(self: &mut GPIO, mut outputs: u32) -> u32 {
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
                for i in 0..self.slowdown_+1 {
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
                for i in 0..self.slowdown_+1 {
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
        //println!("set: {:#X}",*self.gpio_set_bits_);
        //println!("reset: {}",*self.gpio_clr_bits_);
        //println!("value: {}", !value & mask);
        //println!("mask: {}", mask);
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
        let time_value : u32;
        unsafe {
            time_value = std::ptr::read_volatile(self.timereg);
        }

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

impl Frame {
    fn new(message_matrix: MessageMatrix) -> Frame {
        let empty_pixel: Pixel = Pixel {
            r: 0,
            g: 0,
            b: 0
        };
        //Initialize a Frame object with backgroundPixels
        let mut frame: Frame = Frame {
            pos: 32,
            sd_i: 0,
            slowdown: 20,
            cols: message_matrix.pixels[0].len(),
            img: message_matrix.pixels,
            pixels: vec![vec![empty_pixel; COLUMNS]; ROWS]
        };

        let rows = std::cmp::min(frame.pixels.len(), ROWS);
        let cols = std::cmp::min(frame.pixels[0].len(), COLUMNS);

        // if frame.cols < 32 {
        //     frame.pos = frame.cols;
        // } else {
        //     frame.pos = 32;
        // }

        for i in 0..rows{
            for j in 0..cols{
                let red = frame.img[i][j].r;
                let green = frame.img[i][j].g;
                let blue = frame.img[i][j].b;

                let pix: Pixel = Pixel {
                    r: red,
                    g: green,
                    b: blue
                };

                frame.pixels[i][j] = pix;
            }
        }

        frame
    }

    fn render_next(&mut self) {
        self.pos = (self.pos + 1) % self.cols;
        //first move all columns over to the left by one
        for row in 0..self.pixels.len() {
            for column in 0..self.pixels[0].len() -1{
                self.pixels[row][column] = self.pixels[row][column+1];
            }
        }
        //load in the new pixels
        for row in 0..self.pixels.len() {
            let red = self.img[row][self.pos].r;
            let green = self.img[row][self.pos].g;
            let blue = self.img[row][self.pos].b;

            let pix: Pixel = Pixel {
                r: red,
                g: green,
                b: blue
            };

            self.pixels[row][COLUMNS-1] = pix;
        }
    }

    fn fit_image_to_size(&mut self, background_pixel: Pixel) {
        //Get the width of the current message matrix in the frame
        let message_matrix_width = self.pixels[0].len();

        //If the message matrix is smaller than the frame, extend the message to fit the frame
        if message_matrix_width < COLUMNS {
            //Calculate the difference in width
            let width_difference = COLUMNS - message_matrix_width;

            //Add the padding pixels to each row
            for row_index in 0..self.pixels.len() {
                for _ in 0..width_difference {
                    self.pixels[row_index].push(background_pixel);
                }
            }
        }
    }

}

impl LetterMatrix {
    fn new(width: usize, height: usize, pixels: Vec<Vec<Pixel>>) -> LetterMatrix {
        //Initialize an Image object
        let letter_matrix = LetterMatrix {
            width: width,
            height: height,
            pixels: pixels
        };

        letter_matrix
    }
}

impl MessageMatrix {
    fn new(height: usize, padding: usize, background_pixel: Pixel) -> MessageMatrix {
        //Initialize a message matrix with a set amount of columns column of backgournd pixels
        let starter_pixels = vec![vec![background_pixel; padding]; height];

        let message_matrix = MessageMatrix {
            width: padding,
            height: height,
            pixels: starter_pixels
        };

        message_matrix
    }

    fn append_pixel_matrix(&mut self, letter_matrix: &LetterMatrix) {
        //Append the letterMatrix to the image
        for row_index in 0..self.height {
            //Append each pixel value of the current row in the row of the message matrix
            for i in 0..letter_matrix.pixels[row_index+1].len() {
                self.pixels[row_index].push(letter_matrix.pixels[row_index+1][i]);
            }
        }

        //set the new image heigth and width
        self.height = self.pixels.len();
        self.width = self.pixels[0].len();
    }

    fn add_padding_after_message(&mut self, background_pixel: Pixel, padding_amound_after_message: usize) {
        //Generate a padding matrix of background pixels
        let padding_matrix = vec![vec![background_pixel; padding_amound_after_message]; self.height+1];

        //Create a new lettermatrix with the padding
        let letter_padding_matrix = LetterMatrix::new(padding_amound_after_message, self.height, padding_matrix);

        //Add the padding to the image
        self.append_pixel_matrix(&letter_padding_matrix);
    }
}

impl Pixel {
    fn get_corrected_RGB_pixel(&self) -> Pixel {
        let r = sRGB_to_RGB(self.r);
        let g = sRGB_to_RGB(self.g);
        let b = sRGB_to_RGB(self.b);

        let sRGB_pixel = Pixel {
            r: r,
            g: g,
            b: b
        };

        sRGB_pixel
    }
}




// sudo ./target/debug/led_board "string to display" "foreground colour" "background colour" "padding after msg" "scroll_slowdown"
fn main() {
    let args : Vec<String> = std::env::args().collect();
    let interrupt_received = Arc::new(AtomicBool::new(false));

    // sanity checks
    if nix::unistd::Uid::current().is_root() == false {
        eprintln!("Must run as root to be able to access /dev/mem\nPrepend \'sudo\' to the command");
        std::process::exit(1);
    } else if args.len() < 5 {
        eprintln!("The led board driver expects a message, foreground color and background color");
        std::process::exit(1);
    }

    //Extract the given arguments
    let display_string = &args[1][..];
    let foreground_color = &args[2][..];
    let background_color = &args[3][..];
    let padding_after_message_str = &args[4][..];

    //scroll-slowdown
    let mut scroll_slowdown:f32 = 1.0;
    if args.len() == 6 {
        match args[5].parse::<f32>(){
            Ok(n) => scroll_slowdown = n,
            Err(e) => panic!("Could not parse scroll_speed! Make sure it's a valid number")
        }
    } else {
        println!("Defaulting to scroll-slowdown == 1");
    }

    //Set the message to uppercase
    let display_string_uppercase = display_string.to_ascii_uppercase();

    //split the string in its different letters
    let letters: Vec<char> = display_string_uppercase.chars().collect();    

    //Get the pixelvalues for the foreground and background
    let foreground_pixel: Pixel = get_pixelvalues_from_color(foreground_color);
    let background_pixel: Pixel = get_pixelvalues_from_color(background_color);

    //Convert these pixelvalues to the sRGB color space
    let sRGB_foreground_pixel: Pixel = foreground_pixel.get_corrected_RGB_pixel();
    let sRGB_background_pixel: Pixel = background_pixel.get_corrected_RGB_pixel();

    //Generate an image with a startercolumn
    let mut message_matrix: MessageMatrix = MessageMatrix::new(16, 1, sRGB_background_pixel);

    //For each char to display, get the pixel matrix and append them to the image
    for letter in letters {
        let letter_matrix: LetterMatrix = get_letter_matrix_for_char(letter, sRGB_foreground_pixel, sRGB_background_pixel);
        message_matrix.append_pixel_matrix(&letter_matrix);
    }

    let padding_after_message: usize = padding_after_message_str.parse().unwrap();
    message_matrix.add_padding_after_message(sRGB_background_pixel, padding_after_message);

    //Initialise a new Frame and fill it with the message matrix
    let mut frame: Frame = Frame::new(message_matrix);
    
    //If needed extend the image width to the frame width using the background pixels
    frame.fit_image_to_size(sRGB_background_pixel);


    frame.render_next();

    //Initialize the GPIO struct and the Timer struct
    let mut gpio = GPIO::new(1);
    let timer = Timer::new();

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

    //Render loop
    while interrupt_received.load(Ordering::SeqCst) == false {
        let mut color_and_clk_mask = 0;  // Mask of bits while clocking in.
        color_and_clk_mask |= GPIO_BIT!(PIN_R1) |  GPIO_BIT!(PIN_G1) |  GPIO_BIT!(PIN_B1) |  GPIO_BIT!(PIN_R2) |  GPIO_BIT!(PIN_G2) |  GPIO_BIT!(PIN_B2) |  GPIO_BIT!(PIN_CLK);


        for double_row in 0..ROWS/2 {
            for bit in 0..COLOR_DEPTH {
                for column in 0..COLUMNS {
                    let pixel_top = &frame.pixels[double_row][column];
                    let pixel_bottom = &frame.pixels[ROWS/2 + double_row][column];
                    gpio.write_masked_bits(get_plane_bits(pixel_top, pixel_bottom, bit), color_and_clk_mask as u32);
                    gpio.set_bits(GPIO_BIT!(PIN_CLK));
                }

                //reset clock and colors
                gpio.clear_bits(color_and_clk_mask as u32);

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


}
