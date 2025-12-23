#!/bin/bash
# Git repository setup commands for VEXC

echo "# VEXC" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/idkwhatitshouldbeman/VEXC.git
git push -u origin main

