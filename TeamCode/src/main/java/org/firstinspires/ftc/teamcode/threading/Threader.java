package org.firstinspires.ftc.teamcode.threading;

import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.threading.control.DriveThread;
import org.firstinspires.ftc.teamcode.threading.control.UltroImu;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

public final class Threader {
    private static List<UltroThread> threads;
    private static ScheduledExecutorService service;

    public static void registerThreads() {
        if(threads == null) threads = new ArrayList<>();
        if(service == null) service = Executors.newScheduledThreadPool(10);
    }

    public static void registerAuto() {
        registerThreads();
        registerRunnable(UltroImu.class);
    }
    public static void registerDrive() {
        registerThreads();
        registerRunnable(UltroImu.class);
    }

    public static void destroy() {
        List<Runnable> runnables = service.shutdownNow();
        for(Runnable runnable : runnables) {
            if(!(runnable instanceof UltroThread)) continue;
            if(threads.contains(runnable)) {
                ((UltroThread) runnable).interrupt();
            }
        }
    }

    private static void registerRunnable(Class<? extends UltroThread> clasz) {

        try {
            UltroThread thread = clasz.newInstance();
            if(contains(clasz)) return;
            DeviceMap.getInstance().getTelemetry().addData("Added thread: ", clasz.getName());
            threads.add(thread);
            service.scheduleAtFixedRate(thread, 0, thread.getTime(), thread.getTimeUnit());
        }catch (IllegalAccessException|InstantiationException e) {
            e.printStackTrace();
        }
    }

    private static boolean contains(Class<? extends UltroThread> clasz) {
        for(UltroThread thread : threads) {
            if(thread.getClass() == clasz) {
                DeviceMap.getInstance().getTelemetry().addData("Already loaded thread: ", clasz.getName());
                return true;
            }
        }
        return false;
    }
    public static <T extends UltroThread> T get(Class<T> clasz) {
        for(UltroThread t : threads)
            if(t.getClass() == clasz) return (T) t;
        throw new IllegalStateException("type clasz was not a proper runnable " + clasz);
    }

}
