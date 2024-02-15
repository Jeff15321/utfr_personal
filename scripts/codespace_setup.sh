#!/bin/bash

# Author: Arthur Xu
# Description: Setup Github Codespaces for remote development

# Ask for the user's email
read -p "Enter your email for the SSH key: " user_email

# Update package lists
sudo apt-get update

# Install OpenSSH client
sudo apt-get install openssh-client -y

# Generate a new SSH key with the provided email
ssh-keygen -t ed25519 -C "$user_email" -N "" -f ~/.ssh/id_ed25519

# Start the ssh-agent in the background
eval "$(ssh-agent -s)"

# Add your SSH private key to the ssh-agent
ssh-add ~/.ssh/id_ed25519

# Display message in big red text
echo -e "\e[1;31mPUBLIC KEY BELOW\e[0m"

# Print the SSH public key
cat ~/.ssh/id_ed25519.pub

# Install git
sudo apt install git -y

# Install sudo
sudo apt-get install sudo -y

# Instructions for the user to follow after running the script
echo "Copy the outputted public key and add it to your GitHub account under Settings -> SSH and GPG keys -> New SSH key."
echo "https://github.com/settings/keys"