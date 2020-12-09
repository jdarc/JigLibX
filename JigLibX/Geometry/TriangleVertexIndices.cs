namespace JigLibX.Geometry
{
    public struct TriangleVertexIndices
    {
        public int I0, I1, I2;

        public TriangleVertexIndices(int i0, int i1, int i2)
        {
            I0 = i0;
            I1 = i1;
            I2 = i2;
        }

        public void Set(int i0, int i1, int i2)
        {
            I0 = i0;
            I1 = i1;
            I2 = i2;
        }
    }
}