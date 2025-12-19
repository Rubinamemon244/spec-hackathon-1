import React, { useState } from 'react';
import styles from './IsaacCard.module.css';

/**
 * AssessmentTool Component
 * Framework for student assessment and progress tracking
 */
export const AssessmentTool = React.memo(({ title, questions, onComplete }) => {
  const [currentQuestion, setCurrentQuestion] = useState(0);
  const [answers, setAnswers] = useState({});
  const [completed, setCompleted] = useState(false);

  const handleAnswer = (questionIndex, answer) => {
    const newAnswers = { ...answers, [questionIndex]: answer };
    setAnswers(newAnswers);
  };

  const nextQuestion = () => {
    if (currentQuestion < questions.length - 1) {
      setCurrentQuestion(currentQuestion + 1);
    } else {
      setCompleted(true);
      if (onComplete) {
        onComplete(answers);
      }
    }
  };

  const resetAssessment = () => {
    setCurrentQuestion(0);
    setAnswers({});
    setCompleted(false);
  };

  if (completed) {
    return (
      <div className={styles.card} role="region" aria-labelledby="assessment-title">
        <h3 id="assessment-title">Assessment Complete: {title}</h3>
        <p>Your assessment is complete! You answered {Object.keys(answers).length} questions.</p>
        <button
          className={styles.isaacButton}
          onClick={resetAssessment}
          aria-label="Retake the assessment"
        >
          Retake Assessment
        </button>
      </div>
    );
  }

  const currentQ = questions[currentQuestion];
  const questionId = `question-${currentQuestion}`;

  return (
    <div className={styles.card} role="region" aria-labelledby="assessment-title">
      <h3 id="assessment-title">{title}</h3>
      <div className={styles.content}>
        <h4 id={questionId}>
          Question {currentQuestion + 1} of {questions.length}: {currentQ.question}
        </h4>
        <div aria-labelledby={questionId} role="group">
          {currentQ.options.map((option, idx) => {
            const optionId = `option-${currentQuestion}-${idx}`;
            return (
              <div key={idx} style={{ margin: '8px 0' }}>
                <label htmlFor={optionId} style={{ display: 'flex', alignItems: 'center', cursor: 'pointer' }}>
                  <input
                    id={optionId}
                    type={currentQ.type === 'multiple' ? 'checkbox' : 'radio'}
                    name={`q${currentQuestion}`}
                    value={option}
                    checked={answers[currentQuestion] === option}
                    onChange={() => handleAnswer(currentQuestion, option)}
                    style={{ marginRight: '8px', cursor: 'pointer' }}
                    aria-label={option}
                  />
                  <span>{option}</span>
                </label>
              </div>
            );
          })}
        </div>
        <button
          className={styles.isaacButton}
          onClick={nextQuestion}
          disabled={answers[currentQuestion] === undefined}
          aria-label={currentQuestion === questions.length - 1 ? 'Complete assessment' : 'Go to next question'}
        >
          {currentQuestion === questions.length - 1 ? 'Complete Assessment' : 'Next Question'}
        </button>
      </div>
    </div>
  );
});

/**
 * ProgressTracker Component
 * Tracks student progress through the Isaac modules
 */
export const ProgressTracker = React.memo(({ modules, currentModule }) => {
  return (
    <div className={styles.card} role="region" aria-labelledby="progress-title">
      <h3 id="progress-title">Learning Progress</h3>
      <div className={styles.content} role="list">
        {modules.map((module, index) => (
          <div
            key={index}
            style={{
              display: 'flex',
              alignItems: 'center',
              margin: '8px 0',
              padding: '8px',
              backgroundColor: currentModule === index ? '#e3f2fd' : 'transparent',
              borderRadius: '4px'
            }}
            role="listitem"
            aria-label={`${module.title} - ${module.completed ? 'completed' : 'not completed'}`}
          >
            <span
              style={{
                display: 'inline-block',
                width: '20px',
                height: '20px',
                borderRadius: '50%',
                backgroundColor: module.completed ? '#4caf50' : '#f44336',
                marginRight: '8px'
              }}
              aria-label={module.completed ? 'Completed' : 'Not completed'}
              role="img"
            ></span>
            <span>{module.title}</span>
          </div>
        ))}
      </div>
    </div>
  );
});

/**
 * SelfAssessment Component
 * Provides self-assessment questions for students
 */
export const SelfAssessment = React.memo(({ questions }) => {
  return (
    <div className={styles.note} role="region" aria-labelledby="self-assessment-title">
      <div className={styles.noteIcon} aria-hidden="true">❓</div>
      <div className={styles.noteContent}>
        <h4 id="self-assessment-title">Self-Assessment Questions</h4>
        <ol role="list" aria-label="List of self-assessment questions">
          {questions.map((q, i) => (
            <li key={i} style={{ margin: '8px 0' }} role="listitem">{q}</li>
          ))}
        </ol>
      </div>
    </div>
  );
});