package org.firstinspires.ftc.teamcode.threading;

import org.firstinspires.ftc.teamcode.threading.control.UltroImu;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

public final class Threader {
    private static List<UltroThread> threads;
    private static ScheduledExecutorService service;

    public static void registerThreads() {
        if(service != null) service.shutdown();
        threads = new ArrayList<>();
        service = Executors.newScheduledThreadPool(5, Executors.defaultThreadFactory());

        registerRunnable(UltroImu.class);
    }

    public static void destroy() {
        service.shutdown();
        service = null;
    }

    private static void registerRunnable(Class<? extends UltroThread> clasz) {
        try {
            UltroThread thread = clasz.newInstance();
            threads.add(thread);
            service.scheduleAtFixedRate(thread, 0, thread.getTime(), thread.getTimeUnit());
        }catch (IllegalAccessException|InstantiationException e) {
            e.printStackTrace();
        }
    }

    public static <T extends UltroThread> T get(Class<T> clasz) {
        for(UltroThread t : threads) {
            Class<? extends UltroThread> test = t.getClass();
            if(test == clasz)
                return (T) t;
        }
        throw new IllegalStateException("type clasz was not a proper runnable " + clasz);
    }
}
