import sys

# Check if there are command-line arguments
if len(sys.argv) > 1:
    # Get the arguments after the script name
    arguments = ' '.join(sys.argv[1:])
    print(arguments)
else:
    print("No arguments provided.")

