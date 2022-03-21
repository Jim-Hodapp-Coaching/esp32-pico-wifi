// secrets.rs - stores WiFi secrets like SSID, passphrase, etc
//

use heapless::String;

const SSID: &str = "SSID";
const PASSPHRASE: &str = "PASSPHRASE";

// Non-production Ambi server IP address/port pair for local network development
const NON_PROD_AMBI_IP: [u8; 4] = [0, 0, 0, 0];
const NON_PROD_AMBI_PORT: u16 = 4000;