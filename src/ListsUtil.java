import java.util.ArrayList;
import java.util.List;

public class ListsUtil {

    public static double calcAverageInt(List<Integer> list) {
        return list
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

    public static <T> ArrayList<T> findWorstNPercent(double n, ArrayList<T> list) {
        int size = (int) (n * list.size());
        ArrayList<T> newList = new ArrayList<>();
        for (int i = size; i < list.size(); i++) {
            newList.add(list.get(i));
        }
        return newList;
    }
}
