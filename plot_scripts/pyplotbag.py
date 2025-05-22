import pandas as pd
import matplotlib.pyplot as plt
import argparse

def plot_bag(df, x_col, y_col, title=None):
    """
    Plots a bag plot for the given DataFrame.

    Parameters:
    df (pd.DataFrame): The DataFrame containing the data.
    bag_col (str): The column name representing the bag.
    x_col (str): The column name for the x-axis.
    y_col (str): The column name for the y-axis.
    title (str, optional): The title of the plot. Defaults to None.

    Returns:
    None
    """
    
    # Create a new figure
    plt.figure(figsize=(10, 6))

    # Plot the data
    df_plot = df[[x_col, y_col]].dropna()
    plt.plot(df_plot[x_col], df_plot[y_col])
    
    # Set the title and labels
    if title:
        plt.title(title)
    plt.xlabel(x_col)
    plt.ylabel(y_col)
    
    # Show the plot
    plt.show()

def load_bag(file_path):
    """
    Loads a bag from a file.

    Parameters:
    file_path (str): The path to the file containing the bag data.

    Returns:
    pd.DataFrame: A DataFrame containing the bag data.
    """
    
    # Load the bag data into a DataFrame
    df = pd.read_csv(file_path)
    
    return df

def choose_columns(df):
    """
    Prompts the user to choose columns for the bag plot.

    Parameters:
    df (pd.DataFrame): The DataFrame containing the data.

    Returns:
    tuple: A tuple containing the chosen bag column, x column, and y column.
    """
    
    # Display the columns in the DataFrame
    print("Available columns:")
    for i, col in enumerate(df.columns):
        print(f"{i}: {col}")
    
    # Prompt the user to choose columns
    x_col = int(input("Choose an x column (index): "))
    y_col = int(input("Choose a y column (index): "))
    
    return df.columns[x_col], df.columns[y_col]

def main():
    # Example usage
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="input bag path (folder or filepath) to read from")
    args = parser.parse_args()
    file_path = args.input
    df = load_bag(file_path)
    x_col, y_col = choose_columns(df)
    
    # Plot the bag
    plot_bag(df, x_col, y_col, title='Bag Plot Example')

if __name__ == "__main__":
    main()

