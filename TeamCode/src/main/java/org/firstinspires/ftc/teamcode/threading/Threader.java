package org.firstinspires.ftc.teamcode.threading;

import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.threading.control.DriveThread;
import org.firstinspires.ftc.teamcode.threading.control.UltroImu;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public final class Threader {
    private static List<UltroThread> threads;
    private static ExecutorService service;

    public static void registerThreads() {
        if(threads == null) threads = new ArrayList<>();
        if(service == null) service = Executors.newFixedThreadPool(4);

        for(UltroThread thread : threads) {
            thread.interrupt();
        }
        threads.clear();
    }

    public static void registerAuto() {
        registerThreads();
        registerRunnable(UltroImu.class);
        runThreads();
    }
    public static void registerDrive() {
        registerThreads();
        registerRunnable(UltroImu.class);
        runThreads();
    }

    private static void runThreads() {
        for(UltroThread thread : threads)
            service.submit(thread);
    }
    public static void destroy() {
        if(service == null) return;

        for(UltroThread runnable  : threads)
            runnable.interrupt();

        List<Runnable> runnables = service.shutdownNow();
        System.out.println(runnables.size());
        service = null;

    }

    private static void registerRunnable(Class<? extends UltroThread> clasz) {

        try {
            UltroThread thread = clasz.newInstance();
            if(contains(clasz)) {
                thread.reset();
                return;
            }
            DeviceMap.getInstance().getTelemetry().addData("Added thread: ", clasz.getName());
            threads.add(thread);
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
