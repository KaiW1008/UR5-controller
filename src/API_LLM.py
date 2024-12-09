from API.robot_API import UR5MotionAPI
from transformers import AutoModelForCausalLM, AutoTokenizer

class CodeSuggestionLLM:
    """
    Interface to generate Python code suggestions for UR5MotionAPI.
    """
    def __init__(self, model_name="EleutherAI/gpt-neo-125M"):
        print("Initialization started...")
        self.tokenizer = AutoTokenizer.from_pretrained(model_name)

        if self.tokenizer.pad_token is None:
            self.tokenizer.pad_token = self.tokenizer.eos_token

        self.model = AutoModelForCausalLM.from_pretrained(model_name)
        print("Initialization completed.")

    def generate_code(self, user_input, max_new_tokens=300):
        """
        Generate Python code based on the task description.
        Args:
            user_input (str): Natural language description of the task.
            max_new_tokens (int): Maximum tokens to generate.

        Returns:
            str: Python code as a string.
        """
        prompt = f"""# Python code to control a UR5 robot using UR5MotionAPI.
# UR5MotionAPI provides methods for controlling a robot:
# - move_to_joint_positions(point1, point2, velocity, acceleration): Moves the robot in joint space.
# - move_to_cartesian_positions(start_xyz, end_xyz, linear_velocity, linear_acceleration): Moves the robot in Cartesian space.
# - get_robot_state(): Reads the current state of the robot.
#
# Example 1:
# Task: "Move the robot in joint space from [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] to [1.0, 0.5, 0.2, 0.1, 0.0, 0.0] with velocity 0.2 and acceleration 0.1"
# Python code:
api = UR5MotionAPI()
point1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
point2 = [1.0, 0.5, 0.2, 0.1, 0.0, 0.0]
api.move_to_joint_positions(point1, point2, velocity=0.2, acceleration=0.1)

# Example 2:
# Task: "Move the robot in Cartesian space from [0.5, 0, 0.5] to [0.7, -0.2, 0.3]"
# Python code:
api = UR5MotionAPI()
start_xyz = [0.5, 0, 0.5]
end_xyz = [0.7, -0.2, 0.3]
api.move_to_cartesian_positions(start_xyz, end_xyz, linear_velocity=0.1, linear_acceleration=0.1)

# Example 3:
# Task: "Read the current robot state"
# Python code:
api = UR5MotionAPI()
robot_state = api.get_robot_state()
print("Current robot state:", robot_state)

# Task: "{user_input}"
# Python code:
"""
        inputs = self.tokenizer(prompt, return_tensors="pt", padding=True, truncation=True)
        outputs = self.model.generate(
            inputs["input_ids"],
            attention_mask=inputs["attention_mask"],
            max_new_tokens=max_new_tokens,
            pad_token_id=self.tokenizer.pad_token_id
        )
        code = self.tokenizer.decode(outputs[0], skip_special_tokens=True)
        print(f"Generated Code:\n{code}")   
        return code

def main():

    llm = CodeSuggestionLLM()

    print("Welcome to UR5MotionAPI Co-Pilot!")
    print("Describe your task, and Python code will be generated.")
    print("Type 'exit' to quit.\n")

    while True:
        user_input = input("Describe your task: ")
        if user_input.lower() == "exit":
            print("Goodbye!")
            break

        try:
            print("\nGenerating Code...\n")
            suggestion = llm.generate_code(user_input)
            print("Generated Code:\n")
            print(suggestion)
            
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    main()