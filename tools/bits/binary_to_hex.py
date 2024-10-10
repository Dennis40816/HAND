def binary_to_formatted_hex(binary_value):
    """
    Convert a binary string to a formatted hexadecimal string.
    The binary string is converted to hexadecimal and returned as a formatted string.
    
    :param binary_value: Binary value as a string (e.g., "0b10101010").
    :return: Formatted hexadecimal string (e.g., "0xAA").
    """
    # Remove the "0b" prefix and convert binary to integer
    integer_value = int(binary_value, 2)
    
    # Convert integer to hexadecimal string and ensure uppercase format
    hex_str = hex(integer_value).upper().replace('0X', '0x')
    
    return hex_str

def main():
    """
    Main loop to repeatedly prompt the user for a binary value,
    convert it to a formatted hexadecimal string, and print the result.
    The loop can be exited using Ctrl+C.
    """
    print("Enter binary values (e.g., 0b10101010). Press Ctrl+C to exit.")
    
    try:
        while True:
            # Prompt user for input
            binary_value = input("Enter a binary value: ")
            
            # Validate and clean input
            if binary_value.startswith("0b"):
                binary_value = binary_value[2:]
            
            # Ensure the input is binary
            if not all(char in '01' for char in binary_value):
                print("Invalid input! Please enter a valid binary number.\n")
                continue
            
            # Add back "0b" prefix for conversion
            binary_value = '0b' + binary_value
            
            # Convert to formatted hexadecimal and print the result
            formatted_hex = binary_to_formatted_hex(binary_value)
            print(f"Binary: {binary_value} -> Formatted Hex: {formatted_hex}\n")
    
    except KeyboardInterrupt:
        # Handle Ctrl+C to gracefully exit the loop
        print("\nExiting...")

if __name__ == "__main__":
    main()
