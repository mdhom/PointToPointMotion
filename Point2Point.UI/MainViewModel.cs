namespace Point2Point.UI
{
    public class MainViewModel
    {
        public P2PCalculatorViewModel P2PCalculator { get; } = new P2PCalculatorViewModel();
        public JointMotionProfileViewModel JointMotion { get; } = new JointMotionProfileViewModel();
    }
}
