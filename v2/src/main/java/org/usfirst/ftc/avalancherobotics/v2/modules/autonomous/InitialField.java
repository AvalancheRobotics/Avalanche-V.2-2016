package org.usfirst.ftc.avalancherobotics.v2.modules.autonomous;

/**
 * Created by austinzhang on 8/12/16.
 */
public class InitialField {

    public static Cell[][] generateField() {
        Cell[][] field = new Cell[368][368];

        //First set all cells as traversable
        for (int x = 0; x < field.length; x++) {
            for (int y = 0; y < field[0].length; y++) {
                field[x][y] = new Cell(x, y, true);
            }
        }

        //Mark the boarders as off limits
        for (int i = 0; i < field.length; i++) {
            field[0][i] = new Cell(0, i, false);
            field[i][0] = new Cell(i, 0, false);
            field[field.length - 1][i] = new Cell(field.length - 1, i, false);
            field[i][field.length - 1] = new Cell(i, field.length - 1, false);
        }

        /** ADD MORE OBSTACLES WHEN GAME IS ANNOUNCED**/
        return field;
    }
}
