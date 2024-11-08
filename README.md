
# Multi-Agent Cloth Manipulation

This project is a multi-agent simulation for cloth manipulation using PyBullet. It demonstrates a control system where multiple robots work together to deform and manipulate a cloth by grasping and moving it along specified paths. This is useful for research in robotics, multi-agent systems, and cloth simulation.

## Installation

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/multi-agent-cloth-manipulation.git
   cd multi-agent-cloth-manipulation
   ```

2. **Install Dependencies**:
   Ensure you have Python 3.6+ installed, then install the dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Usage

To run the simulation:

```bash
python src/main.py
```

### Controls

- **Number Keys (1-4)**: Select an individual robot to control.
- **Arrow Keys**: Move the selected robot.
- **"a" Key**: Toggle between controlling a single robot and all robots simultaneously.

## Project Structure

```
multi-agent-cloth-manipulation/
│
├── src/                   # Source code
│   ├── main.py            # Main simulation loop and setup
│   ├── robot.py           # Robot creation and movement control functions
│   ├── cloth.py           # Cloth loading and anchoring functions
│   ├── environment.py     # Environment setup and obstacle management
│   └── utils.py           # Utility functions, e.g., finding nearest cloth nodes
│
├── requirements.txt       # Python dependencies
├── README.md              # Project documentation
└── .gitignore             # Files to ignore
```

## Project Description

This simulation allows you to explore multi-agent control for cloth manipulation. The cloth is anchored to specific points managed by robots, which can be moved individually or as a group, creating a dynamic deformation effect on the cloth.

## License

This project is licensed under the MIT License.