import React, { useState, useEffect } from 'react';
import styles from './IsaacCard.module.css';

/**
 * AssessmentTracker Component
 * Tracks and manages student assessments across Isaac modules
 */
export const AssessmentTracker = ({ moduleId, studentId }) => {
  const [assessments, setAssessments] = useState([]);
  const [currentAssessment, setCurrentAssessment] = useState(null);
  const [progress, setProgress] = useState({});

  // Load assessment data from localStorage or API
  useEffect(() => {
    const savedAssessments = localStorage.getItem(`isaac_assessments_${studentId}`);
    if (savedAssessments) {
      setAssessments(JSON.parse(savedAssessments));
    }

    const savedProgress = localStorage.getItem(`isaac_progress_${studentId}`);
    if (savedProgress) {
      setProgress(JSON.parse(savedProgress));
    }
  }, [studentId]);

  // Save assessment data
  const saveAssessment = (assessmentData) => {
    const updatedAssessments = [...assessments, {
      ...assessmentData,
      id: Date.now(),
      timestamp: new Date().toISOString(),
      moduleId
    }];

    setAssessments(updatedAssessments);
    localStorage.setItem(`isaac_assessments_${studentId}`, JSON.stringify(updatedAssessments));
  };

  // Update progress
  const updateProgress = (module, completionPercentage) => {
    const newProgress = {
      ...progress,
      [module]: {
        ...progress[module],
        completion: completionPercentage,
        lastUpdated: new Date().toISOString()
      }
    };

    setProgress(newProgress);
    localStorage.setItem(`isaac_progress_${studentId}`, JSON.stringify(newProgress));
  };

  return (
    <div className={styles.card} role="region" aria-labelledby="assessment-tracker-title">
      <h3 id="assessment-tracker-title">Assessment Tracker</h3>
      <div className={styles.content}>
        <p>Track your progress through Isaac module assessments.</p>

        <div style={{ marginBottom: '16px' }}>
          <h4>Current Progress</h4>
          <div style={{ display: 'flex', flexDirection: 'column', gap: '8px' }}>
            {Object.entries(progress).map(([module, data]) => (
              <div key={module} style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
                <span>{module}</span>
                <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
                  <div style={{
                    width: '100px',
                    height: '10px',
                    backgroundColor: '#e0e0e0',
                    borderRadius: '5px',
                    overflow: 'hidden'
                  }}>
                    <div
                      style={{
                        width: `${data.completion || 0}%`,
                        height: '100%',
                        backgroundColor: '#4caf50',
                        transition: 'width 0.3s ease'
                      }}
                      aria-label={`Progress: ${data.completion || 0}%`}
                    />
                  </div>
                  <span>{data.completion || 0}%</span>
                </div>
              </div>
            ))}
          </div>
        </div>

        <div>
          <h4>Assessment History</h4>
          <ul style={{ listStyle: 'none', padding: 0 }}>
            {assessments.slice(-5).map((assessment) => (
              <li key={assessment.id} style={{
                padding: '8px',
                borderBottom: '1px solid #eee',
                fontSize: '0.9em'
              }}>
                <div><strong>{assessment.title}</strong></div>
                <div>Score: {assessment.score}/{assessment.total} ({assessment.percentage}%)</div>
                <div>Completed: {new Date(assessment.timestamp).toLocaleDateString()}</div>
              </li>
            ))}
            {assessments.length === 0 && (
              <li style={{ padding: '8px', textAlign: 'center', color: '#666' }}>
                No assessments completed yet.
              </li>
            )}
          </ul>
        </div>
      </div>
    </div>
  );
};

/**
 * ModuleProgress Component
 * Shows progress for a specific Isaac module
 */
export const ModuleProgress = ({ moduleName, lessons, currentLesson, onLessonComplete }) => {
  const [completedLessons, setCompletedLessons] = useState(() => {
    const saved = localStorage.getItem(`isaac_module_${moduleName}_lessons`);
    return saved ? JSON.parse(saved) : [];
  });

  useEffect(() => {
    localStorage.setItem(`isaac_module_${moduleName}_lessons`, JSON.stringify(completedLessons));
  }, [completedLessons, moduleName]);

  const toggleLessonComplete = (lessonIndex) => {
    const newCompleted = [...completedLessons];
    const lessonId = lessons[lessonIndex].id;

    if (newCompleted.includes(lessonId)) {
      // Remove from completed
      const index = newCompleted.indexOf(lessonId);
      newCompleted.splice(index, 1);
    } else {
      // Add to completed
      newCompleted.push(lessonId);
    }

    setCompletedLessons(newCompleted);

    if (onLessonComplete) {
      onLessonComplete(lessonIndex, !completedLessons.includes(lessonId));
    }
  };

  const completionPercentage = Math.round((completedLessons.length / lessons.length) * 100);

  return (
    <div className={styles.card} role="region" aria-labelledby={`module-progress-${moduleName}`}>
      <h3 id={`module-progress-${moduleName}`}>{moduleName} Progress</h3>
      <div className={styles.content}>
        <div style={{ marginBottom: '16px' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '8px' }}>
            <span>Overall Progress</span>
            <span>{completionPercentage}%</span>
          </div>
          <div style={{
            width: '100%',
            height: '20px',
            backgroundColor: '#e0e0e0',
            borderRadius: '10px',
            overflow: 'hidden'
          }}>
            <div
              style={{
                width: `${completionPercentage}%`,
                height: '100%',
                backgroundColor: '#2196f3',
                transition: 'width 0.3s ease'
              }}
              aria-label={`Module completion: ${completionPercentage}%`}
            />
          </div>
        </div>

        <div>
          <h4>Lessons</h4>
          <ol style={{ listStyle: 'none', padding: 0 }}>
            {lessons.map((lesson, index) => {
              const isCompleted = completedLessons.includes(lesson.id);
              const isCurrent = index === currentLesson;

              return (
                <li
                  key={lesson.id}
                  style={{
                    padding: '8px',
                    borderBottom: '1px solid #eee',
                    backgroundColor: isCurrent ? '#e3f2fd' : 'transparent',
                    borderRadius: '4px',
                    marginBottom: '4px'
                  }}
                >
                  <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
                    <div>
                      <span style={{ fontWeight: isCurrent ? 'bold' : 'normal' }}>
                        {index + 1}. {lesson.title}
                      </span>
                      {isCurrent && <span style={{ marginLeft: '8px', fontSize: '0.8em', color: '#2196f3' }}>Current</span>}
                    </div>
                    <button
                      onClick={() => toggleLessonComplete(index)}
                      style={{
                        backgroundColor: isCompleted ? '#4caf50' : '#f44336',
                        color: 'white',
                        border: 'none',
                        borderRadius: '12px',
                        width: '24px',
                        height: '24px',
                        cursor: 'pointer',
                        display: 'flex',
                        alignItems: 'center',
                        justifyContent: 'center'
                      }}
                      aria-label={isCompleted ? `Mark "${lesson.title}" as incomplete` : `Mark "${lesson.title}" as complete`}
                    >
                      {isCompleted ? '✓' : '○'}
                    </button>
                  </div>
                </li>
              );
            })}
          </ol>
        </div>
      </div>
    </div>
  );
};

/**
 * CertificateOfCompletion Component
 * Displays certificate of completion for Isaac modules
 */
export const CertificateOfCompletion = ({ moduleName, studentName, completionDate, score }) => {
  return (
    <div
      className={styles.card}
      style={{
        textAlign: 'center',
        padding: '32px',
        border: '4px solid #ffd700',
        borderRadius: '8px',
        backgroundColor: '#fff8dc',
        maxWidth: '600px',
        margin: '16px auto'
      }}
      role="region"
      aria-labelledby="certificate-title"
    >
      <h3 id="certificate-title" style={{ color: '#333', marginBottom: '32px' }}>Certificate of Completion</h3>

      <div style={{ marginBottom: '24px' }}>
        <p style={{ fontSize: '1.2em', margin: '8px 0' }}><strong>This certifies that</strong></p>
        <p style={{ fontSize: '1.5em', fontWeight: 'bold', margin: '8px 0', color: '#2196f3' }}>{studentName}</p>
        <p style={{ fontSize: '1.2em', margin: '8px 0' }}><strong>has successfully completed</strong></p>
        <p style={{ fontSize: '1.3em', fontWeight: 'bold', margin: '8px 0', color: '#4caf50' }}>{moduleName}</p>
      </div>

      <div style={{ marginBottom: '24px' }}>
        <p style={{ margin: '8px 0' }}>Completion Date: {completionDate}</p>
        <p style={{ margin: '8px 0' }}>Score: {score}%</p>
      </div>

      <div style={{ display: 'flex', justifyContent: 'center', gap: '32px', marginTop: '32px' }}>
        <div>
          <div style={{ borderBottom: '1px solid #333', paddingBottom: '4px' }}>Instructor Signature</div>
        </div>
        <div>
          <div style={{ borderBottom: '1px solid #333', paddingBottom: '4px' }}>Date</div>
        </div>
      </div>
    </div>
  );
};