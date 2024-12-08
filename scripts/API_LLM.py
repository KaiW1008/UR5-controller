from motion_library.ur5_motion_library import UR5MotionAPI
from transformers import AutoModelForCausalLM, AutoTokenizer
import rospy

class CodeSuggestionLLM:
    def __init__(self, model_name="EleutherAI/gpt-neo-125M"):
        print("Initilization started")
        self.tokenizer = AutoTokenizer.from_pretrained(model_name)
        print("Initilizated tokenizer")
        self.model = AutoModelForCausalLM.from_pretrained(model_name)
        print("Initilization done")

    def generate_code(self, user_input, max_new_tokens=100):
        prompt = f"""# Python code to control a UR5 robot using UR5MotionAPI.
# UR5MotionAPI methods:
# - move_to_joint_positions(point1, point2, velocity, acceleration)
# - move_to_cartesian_positions(start_xyz, end_xyz, linear_velocity, linear_acceleration)
# - get_robot_state()
#
# Task: "{user_input}"
# Python code:
"""
        inputs = self.tokenizer(prompt, return_tensors="pt")
        outputs = self.model.generate(
            inputs["input_ids"],
            max_new_tokens=max_new_tokens,
            pad_token_id=self.tokenizer.eos_token_id
        )
        return self.tokenizer.decode(outputs[0], skip_special_tokens=True)

def main():
    print("Program started")
    llm = CodeSuggestionLLM()
    rospy.init_node('ur5_motion_copilot', anonymous=True)
    api = UR5MotionAPI()

    print("Welcome to UR5MotionAPI Co-Pilot!")
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
            print("\nExecuting Code...\n")
            exec(suggestion, {"api": api, "UR5MotionAPI": UR5MotionAPI, "rospy": rospy})
        except Exception as e:
            print(f"Error: {e}")


if __name__ == "__main__":
    main()