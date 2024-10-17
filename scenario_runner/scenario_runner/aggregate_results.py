import pandas as pd

def process_csv(file_path):
    df = pd.read_csv(file_path)

    # Drop duplicate rows for each unique "Trial Number", keeping the first occurrence
    # This is because I don't understand why multiple calls to terminate() occur in the Scenario class
    df = df.drop_duplicates(subset='Trial Number', keep='first')

    # Define columns for which we want to compute averages
    numeric_columns = [
        'Ambulance Arrival Time', 'Total Collisions', 'Collisions involving Ambulance',
        'Blocking Vehicles Encountered'
    ]

    # Compute average for numeric columns
    averages = df[numeric_columns].mean()

    # Count total ambulance arrivals (True) and total trials
    total_trials = df['Trial Number'].nunique()
    ambulance_arrived_count = df['Ambulance Arrived'].sum()  # Assuming 'True' is counted as 1

    results = {
        'Total Trials': total_trials,
        'Ambulance Arrived (Count)': ambulance_arrived_count,
        'Ambulance Arrival Rate (%)': (ambulance_arrived_count / total_trials) * 100,
    }

    # Add averages to the results
    for column, avg in averages.items():
        results[f'Average {column}'] = avg

    return results

def process_csv2(file_path):
    df = pd.read_csv(file_path)

    df = df.drop_duplicates(subset='Trial Number', keep='first')

    numeric_columns = [
        'Ambulance Arrival Time', 'Total Collisions', 'Collisions involving Ambulance',
        'Blocking Vehicles Encountered', 'Number of Benevolent Vehicles',
        'Number of Selfish Vehicles', 'Blocking Vehicles Accepted Slow-Down Requests',
        'Blocking Vehicles Rejected Slow-Down Requests'
    ]

    averages = df[numeric_columns].mean()

    total_trials = df['Trial Number'].nunique()
    ambulance_arrived_count = df['Ambulance Arrived'].sum()  # Assuming 'True' is counted as 1

    results = {
        'Total Trials': total_trials,
        'Ambulance Arrived (Count)': ambulance_arrived_count,
        'Ambulance Arrival Rate (%)': (ambulance_arrived_count / total_trials) * 100,
    }

    # Add averages to the results
    for column, avg in averages.items():
        results[f'Average {column}'] = avg

    return results


if __name__ == '__main__':
    # if command line arg "2" is provided after the script name, call process_csv2, else call process_csv
    import sys

    file_path = sys.argv[1]
    if len(sys.argv) > 2 and sys.argv[2] == '2':
        aggregated_data = process_csv2(file_path)
    else:
        aggregated_data = process_csv(file_path)
    print(aggregated_data)