import React from 'react';

/**
 * IsaacSimViewer Component
 * A basic component for displaying Isaac Sim related content
 */
export const IsaacSimViewer = React.memo(({ children, title = "Isaac Sim Example" }) => {
  return (
    <div className="isaac-sim-viewer" role="region" aria-labelledby="isaac-sim-title">
      <h3 id="isaac-sim-title">{title}</h3>
      <div className="content" tabIndex="0" aria-label={`Content for ${title}`}>
        {children}
      </div>
    </div>
  );
});

/**
 * IsaacROSPipeline Component
 * Component for displaying Isaac ROS pipeline examples
 */
export const IsaacROSPipeline = React.memo(({ children, title = "Isaac ROS Pipeline" }) => {
  return (
    <div className="isaac-ros-pipeline" role="region" aria-labelledby="ros-pipeline-title">
      <h3 id="ros-pipeline-title">{title}</h3>
      <div className="pipeline-content" tabIndex="0" aria-label={`Content for ${title}`}>
        {children}
      </div>
    </div>
  );
});

/**
 * Nav2Config Component
 * Component for displaying Nav2 configuration examples
 */
export const Nav2Config = React.memo(({ children, title = "Nav2 Configuration" }) => {
  return (
    <div className="nav2-config" role="region" aria-labelledby="nav2-config-title">
      <h3 id="nav2-config-title">{title}</h3>
      <div className="config-content" tabIndex="0" aria-label={`Content for ${title}`}>
        {children}
      </div>
    </div>
  );
});