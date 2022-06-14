package frc4388.utility.tuner;

import frc4388.utility.Gains;
import frc4388.utility.tuner.annotations.Controller;
import frc4388.utility.tuner.annotations.GainsField;
import frc4388.utility.tuner.annotations.GainsSetter;
import frc4388.utility.tuner.annotations.Reader;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Parameter;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.BiConsumer;

public class TunerController {
    private final Object baseObject;
    private final Map<String, Method> readers;
    private final Map<String, Method> controllers;
    private Gains gains;
    private Method gainsSetter;

    public TunerController(Object baseObject) {
        this.baseObject = baseObject;
        this.readers = new HashMap<>();
        this.controllers = new HashMap<>();
    }

    public void addReader(String name, Method method) {
        this.readers.put(name, method);
    }

    public void addController(String name, Method method) {
        this.controllers.put(name, method);
    }

    public void setGainsSetter(Method gainsSetter) {
        if(this.gainsSetter == null)
            this.gainsSetter = gainsSetter;
        else
            System.err.println("Extraneous GainsSetter");
    }

    public void setGainsObject(Gains gains) {
        if(this.gains == null)
            this.gains = gains;
        else
            System.err.println("Extraneous Gains");
    }

    // TODO: Get separate names for controller/readers
    public Set<String> getNames() {
        Set<String> names = new HashSet<>(this.controllers.keySet());
        names.addAll(this.readers.keySet());
        return names;
    }

    public double readValue(String value) {
        try {
            if(this.readers.containsKey(value))
                return (Double) this.readers.get(value).invoke(this.baseObject);

            return 0;
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new RuntimeException(e);
        }
    }

    public void setTarget(String value, double input) {
        try {
            if(this.controllers.containsKey(value))
                this.controllers.get(value).invoke(this.baseObject, input);
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new RuntimeException(e);
        }
    }

    public void setGains(double kP, double kI, double kD, double kF) {
        try {
            if(this.gainsSetter == null) return;

            if(this.gains == null)
                this.gains = new Gains(0, 0, 0, 0, 0, 0);

            this.gainsSetter.invoke(this.baseObject, this.gains, kP, kI, kD, kF);
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new RuntimeException(e);
        }
    }

    public Gains getGains() {
        return this.gains != null ? this.gains : (this.gains = new Gains(0, 0, 0, 0, 0, 0));
    }

    /* * --------------------------------------------------------------------------------------------------------------- *
       * -------------- You should probably move the following static code into a more appropriate file ---------------- *
       * --------------------------- Either a factory class or the TunerTablesHandler class ---------------------------- *
       * --------------------------------------------------------------------------------------------------------------- */

    public static Map<String, TunerController> createTunerControllers(Object object) {
        Map<String, TunerController> controllers = new HashMap<>(getAnnotatedMethods(object));
        addAnnotatedFields(object, controllers);

        return controllers;
    }

    private static Map<String, TunerController> getAnnotatedMethods(Object object) {
        Map<String, TunerController> controllers = new HashMap<>();
        Class<?> clazz = object.getClass();

        for(Method method : clazz.getMethods()) {
            Reader reader = method.getAnnotation(Reader.class);
            if(reader != null && verifyMethod(method, double.class))
                addAnnotatedMethod(controllers, reader.id(), object, method, (c, m) -> c.addReader(reader.value(), m));

            Controller controller = method.getAnnotation(Controller.class);
            if(controller != null && verifyParameters(method, double.class))
                addAnnotatedMethod(controllers, controller.id(), object, method, (c, m) -> c.addController(controller.value(), m));

            GainsSetter gainsSetter = method.getAnnotation(GainsSetter.class);
            if(gainsSetter != null && verifyParameters(method, double.class, double.class, double.class, double.class))
                addAnnotatedMethod(controllers, gainsSetter.id(), object, method, TunerController::setGainsSetter);
        }

        return  controllers;
    }

    private static boolean verifyMethod(Method method, Class<?> returnType, Class<?>... types) {
        if(!method.getReturnType().equals(returnType)) return false;

        Parameter[] parameters = method.getParameters();
        if(parameters.length != types.length) return false;

        for(int i = 0; i < parameters.length; i++)
            if(!parameters[i].getType().equals(types[i])) return false;

        return true;
    }

    private static boolean verifyParameters(Method method, Class<?>... types) {
        return verifyMethod(method, void.class, types);
    }

    private static void addAnnotatedMethod(Map<String, TunerController> controllers,String id, Object object,
                                          Method method, BiConsumer<TunerController, Method> action)
    {
        controllers.putIfAbsent(id, new TunerController(object));
        action.accept(controllers.get(id), method);
    }

    private static void addAnnotatedFields(Object object, Map<String, TunerController> controllers) {
        Class<?> clazz = object.getClass();

        Field[] fields = clazz.getFields();
        for(Field field : fields) {
            GainsField gainsField = field.getAnnotation(GainsField.class);
            if(gainsField != null && controllers.containsKey(gainsField.id())) try {
                Gains testGains = (Gains) field.get(object);
                controllers.getOrDefault(gainsField.id(), new TunerController(object)).setGainsObject(testGains);
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
