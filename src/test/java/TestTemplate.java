import static org.junit.Assert.*;
import org.junit.*;

public class TestTemplate{
    @Test
    public void shouldDoMath(){
        int a = 1;
        int b = 1;
        int c = a+b;
        assertEquals(2, c);
    }
}
