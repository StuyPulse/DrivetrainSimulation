package com.stuypulse.robot.util;

import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.util.StopWatch;

public class DelayFilter implements IFilter {

    private final StopWatch timer;
    private final double time;

    private double value;

    public DelayFilter(double time) {
        this.timer = new StopWatch();
        this.time = time;

        value = Double.NaN;
    }

    @Override
    public double get(double next) {
        if (Double.isNaN(value) || timer.getTime() > time) {
            value = next;
            timer.reset();
        }

        return value;
    }

    public static void main(String[] args) {
        IStream counter = new IStream() {
            double value = 0.0;
            
            public double get() {
                return value++;
            }
        };

        IStream perOne = counter.polling(1.0);
        IStream perTwo = perOne.filtered(new DelayFilter(2.0));

        for (;;) {
            System.out.println(perOne.get() + ", " + perTwo.get());
        }
    }
    
}
