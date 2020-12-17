package com.ffmpeg.demo;

import java.util.HashMap;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.SynchronousQueue;

public class Main {
    public static void main(String[] args) {
        A.a();
        B.a();
//        SynchronousQueue
        ConcurrentHashMap concurrentHashMap =  null;
        concurrentHashMap.put("","");
//        ConcurrentLinke dDeque
//        ConcurrentLinkedQueue
        HashMap hashMap = new HashMap();
        hashMap.put("","");
    }
}

class A{
    static void a(){
        System.out.println("a");
    }
}

class B extends A{
    static void a(){
        System.out.println("b");
    }
}
