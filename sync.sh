#!/bin/bash

# OS-specific venv activation definition 
if [[ "$OSTYPE" == "linux-gnu"* ]] || [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS or Linux
    VENV_ACTIVATE="source myvirtualenv/bin/activate"
elif [[ "$OSTYPE" == "cygwin" ]] || [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "win32" ]]; then
    # Windows
    VENV_ACTIVATE="myvirtualenv\Scripts\activate"
else
    echo "Unsupported operating system: $OSTYPE"
    exit 1
fi

# Activate virtual environment
echo "Activating virtual environment..."
$VENV_ACTIVATE

# Pull the latest changes from GitHub
echo "Pulling from GitHub..."
git pull

# Install the latest requirements
echo "Installing latest requirements..."
pip install -r requirements.txt

echo "Project update complete!"
