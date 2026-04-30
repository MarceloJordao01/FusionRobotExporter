#!/bin/bash
# Generic deploy script for Fusion 360 Scripts (macOS/Linux)
# Automatically detects project name from folder

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_NAME="$(basename "$SCRIPT_DIR")"

# Detect OS and set Fusion API path
if [[ "$OSTYPE" == "darwin"* ]]; then
    FUSION_API_PATH="$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts"
else
    FUSION_API_PATH="$HOME/.local/share/autodesk/Autodesk Fusion 360/API/Scripts"
fi

DESTINATION_PATH="$FUSION_API_PATH/$PROJECT_NAME"

echo "=== Fusion 360 Script Deploy ==="
echo "Project: $PROJECT_NAME"
echo "Source:  $SCRIPT_DIR"
echo "Target:  $DESTINATION_PATH"
echo ""

# Check if Fusion API folder exists
if [ ! -d "$FUSION_API_PATH" ]; then
    echo "Error: Fusion 360 API folder not found at $FUSION_API_PATH"
    echo "Is Fusion 360 installed?"
    exit 1
fi

# Remove existing installation
if [ -d "$DESTINATION_PATH" ]; then
    echo "Removing existing installation..."
    rm -rf "$DESTINATION_PATH"
fi

# Copy files
echo "Copying files..."
mkdir -p "$DESTINATION_PATH"
rsync -av \
    --exclude='.git' \
    --exclude='deploy.ps1' \
    --exclude='deploy.sh' \
    --exclude='.gitignore' \
    --exclude='__pycache__' \
    "$SCRIPT_DIR/" "$DESTINATION_PATH/"

echo ""
echo "Deployed successfully!"
echo "Open Fusion 360 -> Shift+S -> $PROJECT_NAME -> Run"
