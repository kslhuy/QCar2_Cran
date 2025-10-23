import pandas as pd
from datetime import datetime


class Save_Exel:
    def __init__(self , lossType ,type_learning ,Type_state_Feedback , weight ,learn_rate, lamda,size_dict,max_epochs) -> None:
        self.params = {
            "LossType": lossType,
            "Type_learning": type_learning,
            "Type_state_Feedback": Type_state_Feedback,
            "Weight_e": weight,
            "learn_rate": learn_rate,
            "lamda": lamda,
            "size_dict": size_dict,
            "max_epochs" : max_epochs 
        }
        self.Print()

    def Save(self):

        self.Add("run_time" , datetime.now().strftime("%Y-%m-%d = %H:%M:%S"))

        # Convert to DataFrame
        df = pd.DataFrame([self.params])

        # Append to an existing file or create a new one if it doesn’t exist
        file_name = "run_parameters.xlsx"
        try:
            # Append to the existing file
            existing_df = pd.read_excel(file_name)
            df = pd.concat([existing_df, df], ignore_index=True)
        except FileNotFoundError:
            pass  # No file yet, so create a new one

        # Save to Excel
        try:
            df.to_excel(file_name, index=False)
        except:
            print("Close the exel file Pls !!")


    
    def Print(self):
        # Print the parameters to the console
        print("Parameters:")
        for key, value in self.params.items():
            print( f" - {key}: {value}")
        print("---------")

    def Add(self, key, value):
        # Add a new key-value pair to the parameters dictionary
        self.params[key] = value







