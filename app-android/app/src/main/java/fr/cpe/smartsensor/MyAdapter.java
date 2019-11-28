package fr.cpe.smartsensor;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import androidx.annotation.NonNull;
import androidx.recyclerview.widget.RecyclerView;

import java.util.ArrayList;

/**
 * Adapter used to populate the RecyclerView
 */
class MyAdapter extends RecyclerView.Adapter<MyHolder> {

    private Context c;
    private ArrayList<Model> models;

    MyAdapter(Context c, ArrayList<Model> models) {
        this.c = c;
        this.models = models;
    }

    @NonNull
    @Override
    public MyHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
        View view = LayoutInflater.from(parent.getContext()).inflate(R.layout.row, null);

        return new MyHolder(view);
    }

    @Override
    public void onBindViewHolder(@NonNull MyHolder holder, int i) {
        holder.mTitle.setText(models.get(i).getTitle());
        holder.mDesc.setText(models.get(i).getDescription());
        holder.myImageView.setImageResource(models.get(i).getImg());
    }

    @Override
    public int getItemCount() {
        return models.size();
    }
}
