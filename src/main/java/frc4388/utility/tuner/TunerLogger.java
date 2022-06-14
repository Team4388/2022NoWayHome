package frc4388.utility.tuner;

import frc4388.utility.tuner.annotations.Loggable;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

public class TunerLogger {
    private final Object baseObject;
    private final Field field;

    private final boolean alwaysActive;

    public TunerLogger(Object baseObject, Field field, boolean alwaysActive) {
        this.baseObject = baseObject;
        this.field = field;
        this.alwaysActive = alwaysActive;
    }

    public double getCurrentValue() {
        try {
            return (double) this.field.get(this.baseObject);
        } catch (IllegalAccessException e) {
            return 0;
        }
    }

    /* * --------------------------------------------------------------------------------------------------------------- *
       * -------------- You should probably move the following static code into a more appropriate file ---------------- *
       * --------------------------- Either a factory class or the TunerTablesHandler class ---------------------------- *
       * --------------------------------------------------------------------------------------------------------------- */

    public static Map<String, TunerLogger> createTunerLoggers(Object object) {
        Map<String, TunerLogger> loggers = new HashMap<>();
        Class<?> clazz = object.getClass();

        Field[] fields = clazz.getFields();
        for(Field field : fields) {
            Loggable loggable = field.getAnnotation(Loggable.class);
            if(loggable != null && double.class.isAssignableFrom(field.getType())) {
                field.setAccessible(true);
                loggers.put(loggable.id(), new TunerLogger(object, field, loggable.alwaysActive()));
            }
        }

        return loggers;
    }
}
