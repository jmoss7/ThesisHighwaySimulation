import csv
import os


def log_trial_results(communication_level, trial_number, total_vehicles, ambulance_arrival_time, ambulance_arrived,
                      total_collisions, ambulance_collisions,
                      benevolent_count=None, selfish_count=None,
                      blocking_vehicles_encountered=None, blocking_accepted=None, blocking_rejected=None,
                      csv_filename='simulation_results.csv'):
    """
    Logs the results of a simulation trial to a CSV file.

    Parameters:
        communication_level (int): The communication level of the simulation, so that we know what to include in header.
        trial_number (int): The trial number.
        total_vehicles (int): The total number of vehicles (including the ambulance).
        ambulance_arrival_time (float): The time it took for the ambulance to reach the destination.
        ambulance_arrived (bool): Whether the ambulance reached the destination.
        total_collisions (float): The total number of collisions.
        ambulance_collisions (float): The number of collisions involving the ambulance.
        blocking_vehicles_encountered (int): Number of blocking vehicles encountered.
        benevolent_count (int): The number of benevolent vehicles. (only L2)
        selfish_count (int): The number of selfish vehicles. (only L2)
        blocking_accepted (int): Number of blocking vehicles that accepted slow-down requests. (only L2)
        blocking_rejected (int): Number of blocking vehicles that rejected slow-down requests. (only L2)
        csv_filename (str): Name of the CSV file to write to. Defaults to 'simulation_results.csv'.
    """
    # Check if the file exists to determine whether to write the header
    file_exists = os.path.isfile(csv_filename)

    # Define the row to be written
    row = [
        trial_number,
        total_vehicles,
        ambulance_arrival_time,
        ambulance_arrived,
        total_collisions,
        ambulance_collisions,
        blocking_vehicles_encountered
    ]
    if communication_level == 2:
        row.append(benevolent_count)
        row.append(selfish_count)
        row.append(blocking_accepted)
        row.append(blocking_rejected)

    # Open the CSV file in append mode
    with open(csv_filename, mode='a', newline='') as file:
        writer = csv.writer(file)

        # If file doesn't exist, write the header
        if not file_exists:
            if communication_level == 2:
                header = [
                    'Trial Number',
                    'Total Number of Vehicles',
                    'Ambulance Arrival Time',
                    'Ambulance Arrived',
                    'Total Collisions',
                    'Collisions involving Ambulance',
                    'Blocking Vehicles Encountered',
                    'Number of Benevolent Vehicles',
                    'Number of Selfish Vehicles',
                    'Blocking Vehicles Accepted Slow-Down Requests',
                    'Blocking Vehicles Rejected Slow-Down Requests'
                ]
            else:
                header = [
                    'Trial Number',
                    'Total Number of Vehicles',
                    'Ambulance Arrival Time',
                    'Ambulance Arrived',
                    'Total Collisions',
                    'Collisions involving Ambulance',
                    'Blocking Vehicles Encountered'
                ]
            writer.writerow(header)

        # Write the data row
        writer.writerow(row)