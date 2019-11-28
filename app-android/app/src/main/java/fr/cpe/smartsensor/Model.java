package fr.cpe.smartsensor;

/**
 * Model holding the data of the sensors with their getters and setters
 */
class Model {

    private String ch, title, description;
    private int img;

    String getTitle() {
        return title;
    }

    void setTitle(String title) {
        this.title = title;
    }

    String getDescription() {
        return description;
    }

    void setDescription(String description) {
        this.description = description;
    }

    int getImg() {
        return img;
    }

    void setImg(int img) {
        this.img = img;
    }

    String getCh() {
        return ch;
    }

    void setCh(String ch) {
        this.ch = ch;
    }
}
