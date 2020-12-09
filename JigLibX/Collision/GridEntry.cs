namespace JigLibX.Collision
{
    internal class GridEntry
    {
        public CollisionSkin Skin;

        public GridEntry Previous;

        public GridEntry Next;

        public int GridIndex;

        public GridEntry()
        {
        }

        public GridEntry(CollisionSkin skin)
        {
            Skin = skin;
            Previous = Next = null;
        }

        public static void RemoveGridEntry(GridEntry entry)
        {
            entry.Previous.Next = entry.Next;

            if (entry.Next != null) entry.Next.Previous = entry.Previous;

            entry.Previous = entry.Next = null;
            entry.GridIndex = -2;
        }

        public static void InsertGridEntryAfter(GridEntry entry, GridEntry prev)
        {
            var next = prev.Next;
            prev.Next = entry;
            entry.Previous = prev;
            entry.Next = next;
            if (next != null) next.Previous = entry;
            entry.GridIndex = prev.GridIndex;
        }
    }
}