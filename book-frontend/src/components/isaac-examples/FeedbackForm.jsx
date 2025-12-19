import React, { useState } from 'react';
import styles from './IsaacCard.module.css';

/**
 * FeedbackForm Component
 * Allows students to provide feedback on Isaac tutorials
 */
export const FeedbackForm = React.memo(({ pageName = "current page" }) => {
  const [feedback, setFeedback] = useState({
    rating: 0,
    comment: '',
    helpful: null
  });
  const [submitted, setSubmitted] = useState(false);
  const [hover, setHover] = useState(0);

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFeedback(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleRating = (rating) => {
    setFeedback(prev => ({
      ...prev,
      rating: rating
    }));
  };

  const handleHelpful = (isHelpful) => {
    setFeedback(prev => ({
      ...prev,
      helpful: isHelpful
    }));
  };

  const handleSubmit = (e) => {
    e.preventDefault();

    // In a real implementation, this would send data to a backend
    console.log('Feedback submitted:', {
      ...feedback,
      page: pageName,
      timestamp: new Date().toISOString()
    });

    setSubmitted(true);

    // Reset form after 3 seconds
    setTimeout(() => {
      setFeedback({ rating: 0, comment: '', helpful: null });
      setSubmitted(false);
    }, 3000);
  };

  if (submitted) {
    return (
      <div
        className={`${styles.card} ${styles.success}`}
        role="status"
        aria-live="polite"
      >
        <h3 id="feedback-success-title">Thank You!</h3>
        <p>Your feedback has been received and will help improve the Isaac educational content.</p>
      </div>
    );
  }

  return (
    <div
      className={styles.card}
      role="region"
      aria-labelledby="feedback-title"
      tabIndex="0"
    >
      <h3 id="feedback-title">Provide Feedback</h3>
      <p>Help us improve this tutorial by sharing your feedback.</p>

      <form onSubmit={handleSubmit} className={styles.content}>
        {/* Helpful/Not Helpful Section */}
        <div style={{ marginBottom: '16px' }}>
          <p><strong>Was this content helpful?</strong></p>
          <div style={{ display: 'flex', gap: '10px' }}>
            <button
              type="button"
              onClick={() => handleHelpful(true)}
              className={`${styles.isaacButton} ${feedback.helpful === true ? 'active' : ''}`}
              style={{
                backgroundColor: feedback.helpful === true ? '#4caf50' : '',
                color: 'white',
                border: 'none',
                padding: '8px 16px',
                borderRadius: '4px',
                cursor: 'pointer'
              }}
              aria-pressed={feedback.helpful === true}
            >
              Yes
            </button>
            <button
              type="button"
              onClick={() => handleHelpful(false)}
              className={`${styles.isaacButton} ${feedback.helpful === false ? 'active' : ''}`}
              style={{
                backgroundColor: feedback.helpful === false ? '#f44336' : '',
                color: 'white',
                border: 'none',
                padding: '8px 16px',
                borderRadius: '4px',
                cursor: 'pointer'
              }}
              aria-pressed={feedback.helpful === false}
            >
              No
            </button>
          </div>
        </div>

        {/* Rating Section */}
        <div style={{ marginBottom: '16px' }}>
          <p><strong>Rate your experience:</strong></p>
          <div style={{ display: 'flex', gap: '5px' }}>
            {[1, 2, 3, 4, 5].map((star) => (
              <button
                key={star}
                type="button"
                onClick={() => handleRating(star)}
                onMouseEnter={() => setHover(star)}
                onMouseLeave={() => setHover(0)}
                className={styles.isaacButton}
                style={{
                  backgroundColor: star <= (feedback.rating || hover) ? '#ffc107' : '#e0e0e0',
                  color: star <= (feedback.rating || hover) ? '#000' : '#666',
                  border: 'none',
                  fontSize: '20px',
                  padding: '5px',
                  cursor: 'pointer',
                  width: '30px',
                  height: '30px',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center'
                }}
                aria-label={`Rate ${star} star${star > 1 ? 's' : ''}`}
              >
                ★
              </button>
            ))}
          </div>
        </div>

        {/* Comment Section */}
        <div style={{ marginBottom: '16px' }}>
          <label htmlFor="feedback-comment">
            <p><strong>Additional comments:</strong></p>
          </label>
          <textarea
            id="feedback-comment"
            name="comment"
            value={feedback.comment}
            onChange={handleChange}
            placeholder="What did you like? What could be improved? Any specific questions?"
            rows="4"
            style={{
              width: '100%',
              padding: '8px',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '14px',
              fontFamily: 'inherit'
            }}
            aria-describedby="feedback-hint"
          />
          <div id="feedback-hint" style={{ fontSize: '0.8em', color: '#666', marginTop: '4px' }}>
            Please be specific about what you found helpful or challenging.
          </div>
        </div>

        {/* Submit Button */}
        <button
          type="submit"
          className={styles.isaacButton}
          disabled={feedback.rating === 0 && feedback.helpful === null}
          style={{
            backgroundColor: feedback.rating === 0 && feedback.helpful === null ? '#cccccc' : '#2196f3',
            color: 'white',
            border: 'none',
            padding: '10px 16px',
            borderRadius: '4px',
            cursor: feedback.rating === 0 && feedback.helpful === null ? 'not-allowed' : 'pointer'
          }}
          aria-label="Submit feedback"
        >
          Submit Feedback
        </button>
      </form>
    </div>
  );
});

/**
 * FeedbackPrompt Component
 * A simple prompt to encourage feedback
 */
export const FeedbackPrompt = React.memo(({ title = "How did you find this tutorial?", onFeedback }) => {
  return (
    <div
      className={styles.note}
      role="region"
      aria-labelledby="feedback-prompt-title"
    >
      <div className={styles.noteIcon} aria-hidden="true">💬</div>
      <div className={styles.noteContent}>
        <h4 id="feedback-prompt-title">{title}</h4>
        <p>Your feedback helps us improve the educational content for future students.</p>
        {onFeedback && (
          <button
            className={styles.isaacButton}
            onClick={onFeedback}
            style={{ marginTop: '8px' }}
          >
            Provide Feedback
          </button>
        )}
      </div>
    </div>
  );
});