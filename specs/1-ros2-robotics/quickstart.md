# Quickstart: ROS 2 for Humanoid Robotics Education

## Prerequisites
- Node.js (v18 or higher)
- npm or yarn package manager
- Git for version control

## Setup Instructions

1. **Install Docusaurus**:
   ```bash
   npm init docusaurus@latest docs classic
   ```

2. **Navigate to the project directory**:
   ```bash
   cd docs
   ```

3. **Install dependencies**:
   ```bash
   npm install
   ```

4. **Create the module structure**:
   ```bash
   mkdir -p docs/module-1-ros2-fundamentals
   mkdir -p docs/module-1-ros2-communication
   mkdir -p docs/module-1-urdf-modeling
   ```

5. **Start the development server**:
   ```bash
   npm start
   ```

## Content Creation Workflow

1. **Create module content files** in the appropriate directories
2. **Update sidebar configuration** in `sidebars.js`
3. **Add code examples** with proper syntax highlighting
4. **Test locally** before committing changes

## Configuration Files

- `docusaurus.config.js` - Main site configuration
- `sidebars.js` - Navigation structure
- `package.json` - Dependencies and scripts

## Deployment

The site can be built and deployed to GitHub Pages with:

```bash
npm run build
```

The output will be in the `build/` directory and can be served statically.