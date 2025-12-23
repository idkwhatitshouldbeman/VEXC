# VEX Extension Setup for Cursor

## Installing the VEX Visual Studio Code Extension

Since Cursor is based on VS Code, you can install VS Code extensions directly in Cursor.

### Method 1: Through Cursor UI (Recommended)

1. **Open Cursor Extensions Marketplace:**
   - Click on the Extensions icon in the Activity Bar (left sidebar)
   - Or press `Ctrl+Shift+X` (Linux/Windows) or `Cmd+Shift+X` (Mac)

2. **Search for VEX Extension:**
   - In the search bar, type: `VEX` or `VEX Robotics`
   - Look for "VEX Visual Studio Code Extension" (published by VEX Robotics)

3. **Install the Extension:**
   - Click the "Install" button next to the VEX extension

### Method 2: Via Command Line

Try installing via command line:
```bash
cursor --install-extension VEXRobotics.vex
```

### Method 3: Manual Installation (if marketplace unavailable)

If the extension isn't available through Cursor's marketplace:

1. **Download the extension:**
   - Visit: https://marketplace.visualstudio.com/items?itemName=VEXRobotics.vex
   - Click "Download Extension" to get the `.vsix` file

2. **Install in Cursor:**
   - Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac) to open Command Palette
   - Type: `Extensions: Install from VSIX...`
   - Select the downloaded `.vsix` file

### Verification

After installation, you should be able to:
- Create new VEX robot projects
- Access VEX API IntelliSense
- Use VEX-specific commands via Command Palette (`F1`)

## Notes

- The VEX Visual Studio Code Extension has replaced VEXcode Pro V5
- Requires VS Code/Cursor version 1.66 or later
- The extension provides support for VEX hardware programming

## Resources

- VEX Knowledge Base: https://kb.vex.com
- VS Code Marketplace: https://marketplace.visualstudio.com

