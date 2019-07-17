import java.util.List;

public class AverageUtil {

    public static int calcAverageInt(List<Integer> list) {
        return (int) list
                .stream()
                .mapToInt(a -> a)
                .average()
                .orElse(0);
    }

    public static double calcAverageDouble(List<Double> list) {
        return list
                .stream()
                .mapToDouble(a -> a)
                .average()
                .orElse(0.0);
    }
}
