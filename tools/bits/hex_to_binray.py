def hex_to_formatted_binary(hex_value):
    """
    Convert a 16-bit hexadecimal value to a formatted binary string.
    Each group of 4 bits is separated by a space.
    
    :param hex_value: Hexadecimal value as a string (e.g., "0x1F4A").
    :return: Formatted binary string with groups of 4 bits.
    """
    # Remove the "0x" prefix and convert to binary, ensuring a 16-bit width.
    binary_str = bin(int(hex_value, 16))[2:].zfill(16)

    # Format binary string to have a space every 4 bits.
    formatted_binary = ' '.join([binary_str[i:i+4] for i in range(0, len(binary_str), 4)])
    
    return f"0b {formatted_binary}"

def main():
    """
    Main loop to repeatedly prompt the user for a hexadecimal value,
    convert it to a formatted binary string, and print the result.
    The loop can be exited using Ctrl+C.
    """
    print("Enter 16-bit hexadecimal values (e.g., 0x1F4A). Press Ctrl+C to exit.")
    
    try:
        while True:
            # Prompt user for input.
            hex_value = input("Enter a hexadecimal value: ")
            
            # Convert to formatted binary and print the result.
            formatted_binary = hex_to_formatted_binary(hex_value)
            print(f"Hex: {hex_value} -> Formatted Binary: {formatted_binary}\n")
    
    except KeyboardInterrupt:
        # Handle Ctrl+C to gracefully exit the loop.
        print("\nExiting...")

if __name__ == "__main__":
    main()
