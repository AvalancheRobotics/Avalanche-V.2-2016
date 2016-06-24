package org.usfirst.ftc.avalancherobotics.v2.Modules;

import android.app.Application;
import android.content.Context;

/**
 * Needed to get Context for saving auto
 */
public class MyApplication extends Application {

    private static Context mContext;

    @Override
    public void onCreate() {
        super.onCreate();
        mContext = getApplicationContext();
    }

    public static Context getContext() {
        return mContext;
    }
}
