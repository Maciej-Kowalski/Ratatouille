Project structure:
- legacy firmware - any arduino or stm32 code you used to test your bits without the whole Maciej's data pipeline that you want in the cloud
- legacy - your current code that is not integrated into Maciej's pipeline
- pipelines - containing classes encapsulating the whole mouse prototype (get data from USB, process it somehow, perform some action)
- filters - classes for IMU processing. The class constructor needs to accept all the parameters for your filter as well as two queues - one for receiving packets, one for passing processed data to a function controlling the action (various cursor moving, plotting data)
- audio - same as above for mic
- prototypes - contain working instantiations of the mouse with a given settings (e.g. just read IMU data, ekf,mapping) that can be used for testing/demo

Project Collaboration Guide

Getting Started
    Prerequisites:
    - Git (version control)
    - Python 3.11 + Poetry (dependency management)
    - A GitHub account
    Initial Setup
        Clone the Repository
            #Clone the project to your local machine
            git clone https://github.com/your-username/project-name.gitcd project-name
        Install Poetry (If not already installed)
            pip install poetry

Workflow template
    # First, ensure you're in the project root directory
    # 1. Check current Poetry environment
    poetry env list
    # 2. Activate the poetry environment (if not already active)
    poetry shell
    # 3. Update dependencies BEFORE pulling (ensures local .toml changes are applied first)
    poetry install
    # 4. Pull latest changes
    git pull origin main
    # 5. Update dependencies again (in case pyproject.toml changed)
    poetry install
    # 6. Create and switch to a new feature 
    branchgit checkout -b feature/improved-data-processor#
    # Make your changes 
    # Work on your code... 
    git add . 
    git commit -m "Enhance data processing with new filtering method"  
    # Push the branch to GitHub 
    git push -u origin feature/improved-data-processor
    """Create a Pull Request:
        - Go to GitHub repository
        - Click "Pull Requests"
        - Select your branch
        - Describe your changes
        - Request review from team members
    """

Common Git Commands (feel free to update what you found useful)
    #Stage your changes
    git add .
    #Commit with a descriptive message
    git commit -m "Add detailed description of your changes"
    #Push Your Branch
    git push -u origin feature/your-feature-name

Common Poetry Commands
    #Add a new production dependency
    poetry add numpy pandas
    #Add a development dependency
    poetry add --group dev pytest mypy
    #Update dependencies
    poetry update
    #Remove a dependency
    poetry remove numpy
    #Run scripts defined in pyproject.toml
    poetry run python your_script.py
    #Run tests
    poetry run pytest


Signed Team members:
- Jia Yi Khoo
- Joey Surapakdi
-
-
- Maciej Kowalski
