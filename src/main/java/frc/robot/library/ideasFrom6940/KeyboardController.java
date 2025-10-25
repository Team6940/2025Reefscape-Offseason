package frc.robot.library.ideasFrom6940;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Level;
import java.util.logging.Logger;
import com.github.kwhat.jnativehook.GlobalScreen;
import com.github.kwhat.jnativehook.NativeHookException;
import com.github.kwhat.jnativehook.keyboard.NativeKeyEvent;
import com.github.kwhat.jnativehook.keyboard.NativeKeyListener;

public class KeyboardController implements NativeKeyListener {

    private static final ConcurrentHashMap<Integer, Boolean> keyStates = new ConcurrentHashMap<>();

    /**Global switch for enabling keyboard control.*/
    public static volatile boolean enabled = false;

    public KeyboardController() {
        try {
            Logger logger = Logger.getLogger(GlobalScreen.class.getPackage().getName());
            logger.setLevel(Level.OFF);

            GlobalScreen.registerNativeHook();
            GlobalScreen.addNativeKeyListener(this);
        } catch (NativeHookException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void nativeKeyPressed(NativeKeyEvent e) {
        if (enabled) {
            keyStates.put(e.getKeyCode(), true);
        }
    }

    @Override
    public void nativeKeyReleased(NativeKeyEvent e) {
        if (enabled) {
            keyStates.put(e.getKeyCode(), false);
        }
    }

    @Override
    public void nativeKeyTyped(NativeKeyEvent e) {
        // uneeded
    }

    /** detect whether key is pressed.
     * 
     * @param keyCode
     * @return true if the key is pressed and the keyboard controller is enabled.
     */
    public static boolean isKeyDown(int keyCode) {
        return enabled && keyStates.getOrDefault(keyCode, false);
    }

    public static void enable() {
        enabled = true;
    }

    public static void disable() {
        enabled = false;
        keyStates.clear(); //clear states to avoid residue
    }

    public static boolean isEnabled() {
        return enabled;
    }

    // methods for common keys
    public Trigger esc()   { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_ESCAPE)); }
    public Trigger f1()    { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_F1)); }
    public Trigger f2()    { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_F2)); }
    public Trigger f3()    { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_F3)); }
    public Trigger f4()    { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_F4)); }
    public Trigger f5()    { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_F5)); }
    public Trigger f6()    { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_F6)); }
    public Trigger f7()    { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_F7)); }
    public Trigger f8()    { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_F8)); }
    public Trigger f9()    { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_F9)); }
    public Trigger f10()   { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_F10)); }
    public Trigger f11()   { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_F11)); }
    public Trigger f12()   { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_F12)); }
    public Trigger delete(){ return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_DELETE)); }
    public Trigger backtick() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_BACKQUOTE)); }

    public Trigger one()   { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_1)); }
    public Trigger two()   { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_2)); }
    public Trigger three() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_3)); }
    public Trigger four()  { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_4)); }
    public Trigger five()  { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_5)); }
    public Trigger six()   { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_6)); }
    public Trigger seven() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_7)); }
    public Trigger eight() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_8)); }
    public Trigger nine()  { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_9)); }
    public Trigger zero()  { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_0)); }
    public Trigger minus() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_MINUS)); }
    public Trigger equals(){ return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_EQUALS)); }

    public Trigger backspace() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_BACKSPACE)); }
    public Trigger tab()       { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_TAB)); }
    public Trigger enter()     { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_ENTER)); }
    public Trigger shift()     { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_SHIFT)); }
    public Trigger ctrl()      { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_CONTROL)); }
    public Trigger alt()       { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_ALT)); }

    public Trigger q() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_Q)); }
    public Trigger w() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_W)); }
    public Trigger e() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_E)); }
    public Trigger r() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_R)); }
    public Trigger t() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_T)); }
    public Trigger y() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_Y)); }
    public Trigger u() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_U)); }
    public Trigger i() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_I)); }
    public Trigger o() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_O)); }
    public Trigger p() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_P)); }
    public Trigger a() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_A)); }
    public Trigger s() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_S)); }
    public Trigger d() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_D)); }
    public Trigger f() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_F)); }
    public Trigger g() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_G)); }
    public Trigger h() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_H)); }
    public Trigger j() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_J)); }
    public Trigger k() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_K)); }
    public Trigger l() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_L)); }
    public Trigger z() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_Z)); }
    public Trigger x() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_X)); }
    public Trigger c() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_C)); }
    public Trigger v() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_V)); }
    public Trigger b() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_B)); }
    public Trigger n() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_N)); }
    public Trigger m() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_M)); }

    public Trigger semicolon()   { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_SEMICOLON)); }
    public Trigger apostrophe()  { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_QUOTE)); }
    public Trigger comma()       { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_COMMA)); }
    public Trigger period()      { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_PERIOD)); }

    public Trigger left()  { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_LEFT)); }
    public Trigger right() { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_RIGHT)); }
    public Trigger up()    { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_UP)); }
    public Trigger down()  { return new Trigger(() -> isKeyDown(NativeKeyEvent.VC_DOWN)); }

    private static final Map<String, Integer> keyMap = new HashMap<>();
    static {
        keyMap.put("esc", NativeKeyEvent.VC_ESCAPE);
        keyMap.put("f1", NativeKeyEvent.VC_F1);
        keyMap.put("f2", NativeKeyEvent.VC_F2);
        keyMap.put("f3", NativeKeyEvent.VC_F3);
        keyMap.put("f4", NativeKeyEvent.VC_F4);
        keyMap.put("f5", NativeKeyEvent.VC_F5);
        keyMap.put("f6", NativeKeyEvent.VC_F6);
        keyMap.put("f7", NativeKeyEvent.VC_F7);
        keyMap.put("f8", NativeKeyEvent.VC_F8);
        keyMap.put("f9", NativeKeyEvent.VC_F9);
        keyMap.put("f10", NativeKeyEvent.VC_F10);
        keyMap.put("f11", NativeKeyEvent.VC_F11);
        keyMap.put("f12", NativeKeyEvent.VC_F12);

        keyMap.put("delete", NativeKeyEvent.VC_DELETE);
        keyMap.put("`", NativeKeyEvent.VC_BACKQUOTE);
        keyMap.put("1", NativeKeyEvent.VC_1);
        keyMap.put("2", NativeKeyEvent.VC_2);
        keyMap.put("3", NativeKeyEvent.VC_3);
        keyMap.put("4", NativeKeyEvent.VC_4);
        keyMap.put("5", NativeKeyEvent.VC_5);
        keyMap.put("6", NativeKeyEvent.VC_6);
        keyMap.put("7", NativeKeyEvent.VC_7);
        keyMap.put("8", NativeKeyEvent.VC_8);
        keyMap.put("9", NativeKeyEvent.VC_9);
        keyMap.put("0", NativeKeyEvent.VC_0);
        keyMap.put("-", NativeKeyEvent.VC_MINUS);
        keyMap.put("=", NativeKeyEvent.VC_EQUALS);

        keyMap.put("backspace", NativeKeyEvent.VC_BACKSPACE);
        keyMap.put("tab", NativeKeyEvent.VC_TAB);
        keyMap.put("enter", NativeKeyEvent.VC_ENTER);
        keyMap.put("shift", NativeKeyEvent.VC_SHIFT);
        keyMap.put("ctrl", NativeKeyEvent.VC_CONTROL);
        keyMap.put("alt", NativeKeyEvent.VC_ALT);

        keyMap.put("q", NativeKeyEvent.VC_Q);
        keyMap.put("w", NativeKeyEvent.VC_W);
        keyMap.put("e", NativeKeyEvent.VC_E);
        keyMap.put("r", NativeKeyEvent.VC_R);
        keyMap.put("t", NativeKeyEvent.VC_T);
        keyMap.put("y", NativeKeyEvent.VC_Y);
        keyMap.put("u", NativeKeyEvent.VC_U);
        keyMap.put("i", NativeKeyEvent.VC_I);
        keyMap.put("o", NativeKeyEvent.VC_O);
        keyMap.put("p", NativeKeyEvent.VC_P);
        keyMap.put("a", NativeKeyEvent.VC_A);
        keyMap.put("s", NativeKeyEvent.VC_S);
        keyMap.put("d", NativeKeyEvent.VC_D);
        keyMap.put("f", NativeKeyEvent.VC_F);
        keyMap.put("g", NativeKeyEvent.VC_G);
        keyMap.put("h", NativeKeyEvent.VC_H);
        keyMap.put("j", NativeKeyEvent.VC_J);
        keyMap.put("k", NativeKeyEvent.VC_K);
        keyMap.put("l", NativeKeyEvent.VC_L);
        keyMap.put("z", NativeKeyEvent.VC_Z);
        keyMap.put("x", NativeKeyEvent.VC_X);
        keyMap.put("c", NativeKeyEvent.VC_C);
        keyMap.put("v", NativeKeyEvent.VC_V);
        keyMap.put("b", NativeKeyEvent.VC_B);
        keyMap.put("n", NativeKeyEvent.VC_N);
        keyMap.put("m", NativeKeyEvent.VC_M);

        keyMap.put(";", NativeKeyEvent.VC_SEMICOLON);
        keyMap.put("'", NativeKeyEvent.VC_QUOTE);
        keyMap.put(",", NativeKeyEvent.VC_COMMA);
        keyMap.put(".", NativeKeyEvent.VC_PERIOD);

        keyMap.put("left", NativeKeyEvent.VC_LEFT);
        keyMap.put("right", NativeKeyEvent.VC_RIGHT);
        keyMap.put("up", NativeKeyEvent.VC_UP);
        keyMap.put("down", NativeKeyEvent.VC_DOWN);
    }

    /** Publish key state to smartDashboard */
    public static void publishKeyboard() {
        boolean enabled = SmartDashboard.getBoolean("keyboard/enabled", false);

        for (Map.Entry<String, Integer> entry : keyMap.entrySet()) {
            String name = entry.getKey();
            int code = entry.getValue();
            SmartDashboard.putBoolean("keyboard/" + name, isKeyDown(code));
        }

        if (enabled == true) {
            KeyboardController.enable();           
            for (Map.Entry<String, Integer> entry : keyMap.entrySet()) {
                String name = entry.getKey();
                int code = entry.getValue();
                boolean pressed = SmartDashboard.getBoolean("keyboard/" + name, false);
                keyStates.put(code, pressed);
            }
        } else if (enabled == false) {
            KeyboardController.disable();
        }
    }

    // /** Create a virtual global switch button on SmartDashboard. */
    // public static void syncWithDashboard() {
    //     boolean enabled = SmartDashboard.getBoolean("keyboard/enabled", false);
    
    //     if (enabled == true) {
    //         KeyboardController.enable();
    //     } else if (enabled == false) {
    //         KeyboardController.disable();
    //     }

    //     if (enabled) {
    //         for (Map.Entry<String, Integer> entry : keyMap.entrySet()) {
    //             String name = entry.getKey();
    //             int code = entry.getValue();
    //             boolean pressed = SmartDashboard.getBoolean("keyboard/" + name, false);
    //             keyStates.put(code, pressed);
    //         }
    //     }
    // }
}
