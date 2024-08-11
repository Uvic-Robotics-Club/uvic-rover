import os

def parse_lander_log(file_path):
    # Check if the file exists
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"{file_path} does not exist.")

    # Initialize dictionaries to store the status of each cell
    cell_status = {}

    # Read the file and process each line
    with open(file_path, 'r') as file:
        for line in file:
            # Only process lines that are related to fuel cells
            if 'Cell' in line and 'Diagnostic' in line:
                # Split the line into parts
                parts = line.strip().split()
                # Extract cell number and status
                cell_number = parts[3]
                diagnostic_status = parts[5]

                # Store the status of the cell
                cell_status[cell_number] = diagnostic_status

    return cell_status

def check_functioning_cells(cell_status):
    functioning_cells = []
    malfunctioning_cells = []

    # Classify cells based on their status
    for cell_number, status in cell_status.items():
        if status == 'OK':
            functioning_cells.append(cell_number)
        elif status == 'FAIL':
            malfunctioning_cells.append(cell_number)

    return functioning_cells, malfunctioning_cells

# Define the path to the log file
log_file_path = 'lander.log'

# Parse the log file and check the status of the cells
try:
    cell_status = parse_lander_log(log_file_path)
    functioning_cells, malfunctioning_cells = check_functioning_cells(cell_status)

    print(f"Functioning Cells: {functioning_cells}")
    print(f"Malfunctioning Cells: {malfunctioning_cells}")

except Exception as e:
    print(f"An error occurred: {e}")