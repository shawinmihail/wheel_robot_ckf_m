function smoothed = ekf4_smooth_K(sample, smoothed, K)
    smoothed = smoothed + K * (sample - smoothed);
end

